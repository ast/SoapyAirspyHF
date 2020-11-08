/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Charles J. Cliffe
 * Copyright (c) 2018 Corey Stotts
 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyAirspyHF.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/ConverterRegistry.hpp>
#include <cstring> // memcpy
#include <cassert>
#include <chrono>

#define SOAPY_NATIVE_FORMAT SOAPY_SDR_CF32

std::vector<std::string> SoapyAirspyHF::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;
    
    for (const auto &target : SoapySDR::ConverterRegistry::listTargetFormats(SOAPY_NATIVE_FORMAT))
    {
        formats.push_back(target);
    }
    
    return formats;
}

std::string SoapyAirspyHF::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
    fullScale = 1.0;
    return SOAPY_NATIVE_FORMAT;
}

SoapySDR::ArgInfoList SoapyAirspyHF::getStreamArgsInfo(const int direction, const size_t channel) const {
    SoapySDR::ArgInfoList streamArgs;
    return streamArgs;
}

/*******************************************************************
 * Async thread work
 ******************************************************************/

// C trampoline function
static int _rx_callback(airspyhf_transfer_t *t)
{
    SoapyAirspyHF *self = (SoapyAirspyHF *)t->ctx;
    return self->rx_callback(t);
}

int SoapyAirspyHF::rx_callback(airspyhf_transfer_t *t)
{
    std::unique_lock<std::mutex> lock(_stream_mutex);
    while ((_stream_buff == nullptr) && airspyhf_is_streaming(dev))
        _stream_cond.wait(lock);
    _dropped_samples = t->dropped_samples;
    if (t->dropped_samples) {
        _dropped_samples = t->dropped_samples;
    }
    // Convert into stream buffer
    if(_stream_buff != nullptr) {
        converterFunction(t->samples, _stream_buff, t->sample_count, 1.0);
    }
    _stream_buff = nullptr;
    _callback_done_cond.notify_one();
    
    // 0 == success
    return 0;
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapyAirspyHF::setupStream(const int direction,
                                             const std::string &format,
                                             const std::vector<size_t> &channels,
                                             const SoapySDR::Kwargs &args)
{
    //check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0)) {
        throw std::runtime_error("setupStream invalid channel selection");
    }
    
    std::vector<std::string> sources = SoapySDR::ConverterRegistry::listSourceFormats(format);
    
    if (std::find(sources.begin(), sources.end(), SOAPY_NATIVE_FORMAT) == sources.end()) {
        throw std::runtime_error(
                                 "setupStream invalid format '" + format + "'.");
    }
    
    converterFunction = SoapySDR::ConverterRegistry::getFunction(SOAPY_NATIVE_FORMAT, format, SoapySDR::ConverterRegistry::GENERIC);
    
    return (SoapySDR::Stream *) this;
}

void SoapyAirspyHF::closeStream(SoapySDR::Stream *stream)
{
}

size_t SoapyAirspyHF::getStreamMTU(SoapySDR::Stream *stream) const
{
    return _airspyhf_output_size;
}

int SoapyAirspyHF::activateStream(SoapySDR::Stream *stream,
                                  const int flags,
                                  const long long timeNs,
                                  const size_t numElems)
{
    int ret;
    
    SoapySDR::log(SOAPY_SDR_INFO, "activateStream");
    
    if (flags != 0) {
        SoapySDR::log(SOAPY_SDR_FATAL, "SOAPY_SDR_NOT_SUPPORTED");
        return SOAPY_SDR_NOT_SUPPORTED;
    }
    
    _dropped_samples = 0;
    _stream_buff = nullptr;
    
    ret = airspyhf_start(dev, &_rx_callback, (void *) this);
    assert(ret == AIRSPYHF_SUCCESS);
    
    return 0;
}

int SoapyAirspyHF::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    SoapySDR::log(SOAPY_SDR_INFO, "deactivateStream");
    
    if (flags != 0) {
        SoapySDR::log(SOAPY_SDR_FATAL, "SOAPY_SDR_NOT_SUPPORTED");
        return SOAPY_SDR_NOT_SUPPORTED;
    }
    
    airspyhf_stop(dev);
    // make sure we don't get stuck somewhere.
    _stream_cond.notify_all();
    _callback_done_cond.notify_all();
    
    return 0;
}

int SoapyAirspyHF::readStream(SoapySDR::Stream *stream,
                              void * const *buffs,
                              const size_t numElems,
                              int &flags,
                              long long &timeNs,
                              const long timeoutUs)
{
    // TODO: check flags?
    if(!airspyhf_is_streaming(dev)) {
        // strictly speaking need to implement timeout
        return SOAPY_SDR_TIMEOUT;
    }
    
    if (numElems < _airspyhf_output_size) {
        // wait until we get called with more
        return 0;
    }
    
    std::unique_lock<std::mutex> lock(_stream_mutex);
    _stream_buff = buffs[0];
    _dropped_samples = 0;
    // Notify callback that the buffer is ready for samples
    _stream_cond.notify_one();
    
    // Wait for callback to copy, the callback will set this to nullptr when done
    if (timeNs) {
        // Wait with timeout
        auto timeout_duration = std::chrono::nanoseconds(timeNs);
        while ((_stream_buff != nullptr) && airspyhf_is_streaming(dev)) {
            if(_callback_done_cond.wait_for(lock, timeout_duration) == std::cv_status::timeout) {
                SoapySDR::logf(SOAPY_SDR_INFO, "readStream timeout: %d", timeNs);
                return SOAPY_SDR_TIMEOUT;
            }
        }
    } else {
        // Wait without timeout
        while ((_stream_buff != nullptr) && airspyhf_is_streaming(dev)) _callback_done_cond.wait(lock);
    }
    
    if (_dropped_samples) {
        SoapySDR::logf(SOAPY_SDR_INFO, "dropped samples: %d", _dropped_samples);
        return SOAPY_SDR_OVERFLOW;
    }

    return (int) _airspyhf_output_size;
}
