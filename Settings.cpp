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
#include <cassert>
#include <complex>

SoapyAirspyHF::SoapyAirspyHF(const SoapySDR::Kwargs &args):
sampleRate(768000),
centerFrequency(0),
agcMode(1),
lnaGain(0),
rfGain(4),
dev(nullptr)
{
    std::stringstream serialstr;
    
    if (args.count("serial") != 0)
    {
        try {
            serial = std::stoull(args.at("serial"), nullptr, 16);
        } catch (const std::invalid_argument &) {
            throw std::runtime_error("serial is not a hex number");
        } catch (const std::out_of_range &) {
            throw std::runtime_error("serial value of out range");
        }
        serialstr << std::hex << serial;
        if (airspyhf_open_sn(&dev, serial) != AIRSPYHF_SUCCESS) {
            throw std::runtime_error("Unable to open AirspyHF device with S/N " + serialstr.str());
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Found AirspyHF+ device: serial = %16Lx", serial);
    }
    else
    {
        if (airspyhf_open(&dev) != AIRSPYHF_SUCCESS) {
            throw std::runtime_error("Unable to open AirspyHF device");
        }
    }
    
    if (airspyhf_set_hf_att(dev,rfGain) == AIRSPYHF_SUCCESS) {
        airspyhf_set_hf_lna(dev, lnaGain);
        airspyhf_set_hf_agc(dev, agcMode);
    }
    
    // TODO: maybe fix hardcoding
    airspyhf_set_lib_dsp(dev, 1);
    airspyhf_set_hf_agc_threshold(dev, 1);
    
    _airspyhf_output_size = airspyhf_get_output_size(dev);
    
    //apply arguments to settings when they match
    for (const auto &info : this->getSettingInfo())
    {
        const auto it = args.find(info.key);
        if (it != args.end()) this->writeSetting(it->first, it->second);
    }
}

SoapyAirspyHF::~SoapyAirspyHF(void)
{
    airspyhf_close(dev);
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyAirspyHF::getDriverKey(void) const
{
    return "AirspyHF";
}

std::string SoapyAirspyHF::getHardwareKey(void) const
{
    return "AirspyHF";
}

SoapySDR::Kwargs SoapyAirspyHF::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs args;
    std::stringstream serialstr;
    serialstr << std::hex << serial;
    args["serial"] = serialstr.str();
    
    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyAirspyHF::getNumChannels(const int dir) const
{
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyAirspyHF::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;
    antennas.push_back("RX");
    return antennas;
}

void SoapyAirspyHF::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    // Not configurable
    SoapySDR_logf(SOAPY_SDR_INFO, "Antenna not configurable");
}

std::string SoapyAirspyHF::getAntenna(const int direction, const size_t channel) const
{
    // eventually could change this to HF/VHF
    // based on frequency because selection is automatic
    return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyAirspyHF::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return false;
}

bool SoapyAirspyHF::hasIQBalance(const int direction, const size_t channel) const {
    return true;
}

void SoapyAirspyHF::setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance) {
    int ret;
    _iq_correction_w = std::arg(balance);
    
    SoapySDR_logf(SOAPY_SDR_INFO, "Setting IQBalance: %f", _iq_correction_w);
    
    ret = airspyhf_set_optimal_iq_correction_point(dev, _iq_correction_w);
    assert(ret == AIRSPYHF_SUCCESS);
}
std::complex<double> SoapyAirspyHF::getIQBalance(const int direction, const size_t channel) const {
    return std::complex<double>(std::cos(_iq_correction_w), std::sin(_iq_correction_w));
}


/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyAirspyHF::listGains(const int direction, const size_t channel) const
{
    //list available gain elements,
    //the functions below have a "name" parameter
    std::vector<std::string> results;
    results.push_back("LNA");
    results.push_back("RF");
    
    return results;
}

bool SoapyAirspyHF::hasGainMode(const int direction, const size_t channel) const
{
    // We have agc on/off setting or it's forced on, either way AGC is supported
    return true;
}

void SoapyAirspyHF::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting AGC: %s", automatic ? "Automatic" : "Manual");
    airspyhf_set_hf_agc(dev,agcMode=automatic ? 1:0);
}

bool SoapyAirspyHF::getGainMode(const int direction, const size_t channel) const
{
    return agcMode ? true : false; //agc is finally not always on
}

SoapySDR::Range SoapyAirspyHF::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "LNA") return SoapySDR::Range(0,6,6);
    return SoapySDR::Range(-48.0,0,6);
}

double SoapyAirspyHF::getGain(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "LNA") {
        return lnaGain * 6.0;
    }
    else if(name == "RF") {
        return rfGain * 6.0;
    }
    else {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Unknown gain: %s", name.c_str());
        return 0.;
    }
}

void SoapyAirspyHF::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    int ret;
    
    if (name == "LNA") {
        lnaGain = value >= 3.0 ? 1 : 0;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting LNA gain: %f = %d", value, lnaGain);
        ret = airspyhf_set_hf_lna(dev, lnaGain);
        assert(ret == AIRSPYHF_SUCCESS);
    }
    else if(name == "RF") {
        rfGain = std::round(-value/6.0);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting RF gain: %f = %d", value, rfGain);
        ret = airspyhf_set_hf_att(dev, rfGain);
        assert(ret == AIRSPYHF_SUCCESS);
    }
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyAirspyHF::setFrequency(const int direction,
                                 const size_t channel,
                                 const std::string &name,
                                 const double frequency,
                                 const SoapySDR::Kwargs &args)
{
    if (name == "RF")
    {
        centerFrequency = (uint32_t) frequency;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting center freq: %d", centerFrequency);
        airspyhf_set_freq(dev, centerFrequency);
    }
}

double SoapyAirspyHF::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "RF") {
        return (double) centerFrequency;
    } else {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Unknown frequency: %s", name.c_str());
        return 0;
    }
}

std::vector<std::string> SoapyAirspyHF::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

SoapySDR::RangeList SoapyAirspyHF::getFrequencyRange(const int direction,
                                                     const size_t channel,
                                                     const std::string &name) const
{
    SoapySDR::RangeList results;
    if (name == "RF")
    {
        results.push_back(SoapySDR::Range(9e3, 31e6));
        results.push_back(SoapySDR::Range(60e6, 260e6));
    }
    return results;
}

SoapySDR::ArgInfoList SoapyAirspyHF::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;
    
    // TODO: frequency arguments
    
    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyAirspyHF::setSampleRate(const int direction, const size_t channel, const double rate)
{
    int ret;
    
    if (sampleRate != rate) {
        sampleRate = rate;
        ret = airspyhf_set_samplerate(dev, rate);
        assert(ret == AIRSPYHF_SUCCESS);
        int is_low_if = airspyhf_is_low_if(dev);
        
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting sample rate: %d, is low IF: %d",
                      sampleRate, is_low_if);
    }
}

double SoapyAirspyHF::getSampleRate(const int direction, const size_t channel) const
{
    return sampleRate;
}

std::vector<double> SoapyAirspyHF::listSampleRates(const int direction, const size_t channel) const
{
    int ret;
    std::vector<double> results;
    
    uint32_t numRates;
    airspyhf_get_samplerates(dev, &numRates, 0);
    
    std::vector<uint32_t> samplerates;
    samplerates.resize(numRates);
    
    ret = airspyhf_get_samplerates(dev, samplerates.data(), numRates);
    assert(ret == AIRSPYHF_SUCCESS);
    
    for (auto i: samplerates) {
        results.push_back(i);
    }
    
    return results;
}

void SoapyAirspyHF::setBandwidth(const int direction, const size_t channel, const double bw)
{
    SoapySDR::Device::setBandwidth(direction, channel, bw);
}

double SoapyAirspyHF::getBandwidth(const int direction, const size_t channel) const
{
    return SoapySDR::Device::getBandwidth(direction, channel);
}

std::vector<double> SoapyAirspyHF::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> results;
    
    return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyAirspyHF::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;
    return setArgs;
}

void SoapyAirspyHF::writeSetting(const std::string &key, const std::string &value)
{
}

std::string SoapyAirspyHF::readSetting(const std::string &key) const
{
    SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
}

/*******************************************************************
 * Clocking API
 ******************************************************************/
std::vector<std::string> SoapyAirspyHF::listClockSources(void) const {
    std::vector<std::string> sources;
    sources.push_back(getClockSource());
    return sources;
}

std::string SoapyAirspyHF::getClockSource(void) const {
    return "internal";
}

double SoapyAirspyHF::getMasterClockRate(void) const {
    // TODO: fixed but might change with firmware
    return 36.864e6;
}
