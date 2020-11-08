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
#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.h>
#include <SoapySDR/Types.h>
#include <SoapySDR/ConverterRegistry.hpp>
#include <stdexcept>
#include <mutex>
#include <condition_variable>
#include <string>
#include <algorithm>

#include <libairspyhf/airspyhf.h>

#define MAX_DEVICES 32

class SoapyAirspyHF: public SoapySDR::Device
{
public:
    SoapyAirspyHF(const SoapySDR::Kwargs &args);
    ~SoapyAirspyHF(void);
    
    /*******************************************************************
     * Identification API
     ******************************************************************/
    std::string getDriverKey(void) const;
    std::string getHardwareKey(void) const;
    SoapySDR::Kwargs getHardwareInfo(void) const;
    
    /*******************************************************************
     * Channels API
     ******************************************************************/
    size_t getNumChannels(const int) const;
    
    /*******************************************************************
     * Stream API
     ******************************************************************/
    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;
    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;
    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;
    SoapySDR::Stream *setupStream(const int direction, const std::string &format, const std::vector<size_t> &channels =
                                  std::vector<size_t>(), const SoapySDR::Kwargs &args = SoapySDR::Kwargs());
    void closeStream(SoapySDR::Stream *stream);
    size_t getStreamMTU(SoapySDR::Stream *stream) const;
    int activateStream(SoapySDR::Stream *stream,
                       const int flags = 0,
                       const long long timeNs = 0,
                       const size_t numElems = 0);
    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0);
    int readStream(SoapySDR::Stream *stream,
                   void * const *buffs,
                   const size_t numElems,
                   int &flags,
                   long long &timeNs,
                   const long timeoutUs = 100000);
    
    /*******************************************************************
     * Antenna API
     ******************************************************************/
    std::vector<std::string> listAntennas(const int direction, const size_t channel) const;
    void setAntenna(const int direction, const size_t channel, const std::string &name);
    std::string getAntenna(const int direction, const size_t channel) const;
    
    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/
    bool hasDCOffsetMode(const int direction, const size_t channel) const;
    bool hasIQBalance(const int direction, const size_t channel) const;
    void setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance);
    std::complex<double> getIQBalance(const int direction, const size_t channel) const;

    /*******************************************************************
     * Gain API
     ******************************************************************/
    std::vector<std::string> listGains(const int direction, const size_t channel) const;
    bool hasGainMode(const int direction, const size_t channel) const;
    void setGainMode(const int direction, const size_t channel, const bool automatic);
    bool getGainMode(const int direction, const size_t channel) const;
    // void setGain(const int direction, const size_t channel, const double value);
    void setGain(const int direction, const size_t channel, const std::string &name, const double value);
    double getGain(const int direction, const size_t channel, const std::string &name) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const;
    
    /*******************************************************************
     * Frequency API
     ******************************************************************/
    void setFrequency(const int direction,
                      const size_t channel,
                      const std::string &name,
                      const double frequency,
                      const SoapySDR::Kwargs &args = SoapySDR::Kwargs());
    
    double getFrequency(const int direction, const size_t channel, const std::string &name) const;
    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const;
    SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;
    
    /*******************************************************************
     * Sample Rate API
     ******************************************************************/
    void setSampleRate(const int direction, const size_t channel, const double rate);
    double getSampleRate(const int direction, const size_t channel) const;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const;
    void setBandwidth(const int direction, const size_t channel, const double bw);
    double getBandwidth(const int direction, const size_t channel) const;
    std::vector<double> listBandwidths(const int direction, const size_t channel) const;
    
    /*******************************************************************
     * Utility
     ******************************************************************/
    
    /*******************************************************************
     * Clocking API
     ******************************************************************/
    
    /*void setMasterClockRate(const double rate);
    double getMasterClockRate(void) const;
    RangeList getMasterClockRates(void) const;
    void setReferenceClockRate(const double rate);
    double getReferenceClockRate(void) const;
    RangeList getReferenceClockRates(void) const;
    std::vector<std::string> listClockSources(void) const;
    void setClockSource(const std::string &source);
    std::string getClockSource(void) const;*/
    
    /*******************************************************************
     * Settings API
     ******************************************************************/
    SoapySDR::ArgInfoList getSettingInfo(void) const;
    void writeSetting(const std::string &key, const std::string &value);
    std::string readSetting(const std::string &key) const;
    
private:
    // device
    uint64_t serial;
    airspyhf_device_t *dev;
    
    //cached settings
    bool hasgains;
    uint32_t sampleRate, centerFrequency;
    unsigned int bufferLength;
    size_t numBuffers;
    bool rfBias, bitPack;
    uint8_t lnaGain,rfGain, agcMode;
    int bytesPerSample;
    SoapySDR::ConverterRegistry::ConverterFunction converterFunction;
    
    float _iq_correction_w;
    uint32_t _airspyhf_output_size;
    uint64_t _dropped_samples;
    void *_stream_buff;
    std::mutex _stream_mutex;
    std::condition_variable _stream_cond;
    std::condition_variable _callback_done_cond;
    
public:
    // async api
    int rx_callback(airspyhf_transfer_t *t);
};
