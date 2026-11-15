/*
 * File: NTPServer.h
 * Description:
 *   NTP server implementation.
 * Author: Mooneer Salem <mooneer@gmail.com>
 *         Jan-David Förster
 * License: New BSD License
 */

#ifndef NTP_SERVER_H
#define NTP_SERVER_H

#include "NTPTimeBase.h"
#ifdef ESP32
#include "NtpUdp.h"
#endif

// Supplies the time quality, the UTC second and micros() at its start. false = do not answer
typedef bool (*NtpSourceFn)(NtpTimeBase* base, uint32_t* utc, uint32_t* boundaryUs);
typedef void (*NtpSentFn)(const NtpTimeBase &base);

class NtpServer
{
public:
#ifdef ESP32
    NtpServer(WiFiUDP Port) { (void)Port; }   // ESP32 answers from the lwIP callback, no WiFiUDP
#else
    NtpServer(WiFiUDP Port) { timeServerPort_ = Port; }
#endif

    // Must be set before beginListening()
    void setSource(NtpSourceFn source, NtpSentFn sent)
    {
        source_ = source;
        sent_ = sent;
    }

    bool beginListening(void);
    void stopListening(void);        // Release port 123

#ifdef ESP32
    void poll(void) {}               // Replies are sent in NtpRecv()
#else
    // Answer one request. microsecs: micros() at the start of the second
    bool processOneRequest(uint32_t utc, uint32_t microsecs, const NtpTimeBase &base);
    void poll(void);                 // Answer or drop one request with the time from source_
#endif

    NtpSourceFn source_ = nullptr;   // Public for NtpRecv()
    NtpSentFn sent_ = nullptr;
#ifdef ESP32
    NtpUdp udp_;
#else
private:
    WiFiUDP timeServerPort_;
#endif
};

#endif // NTP_SERVER_H
