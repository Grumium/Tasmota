/*
 * File: NTPServer.h
 * Description:
 *   NTP server implementation.
 * Author: Mooneer Salem <mooneer@gmail.com>
 *         Jan-David Förster and Claude
 * License: New BSD License
 */

#ifndef NTP_SERVER_H
#define NTP_SERVER_H

#include "NTPTimeBase.h"
#ifdef ESP32
#include "NtpUdp.h"
#endif

// Get time quality, second and its start in us. false = do not answer
typedef bool (*NtpSourceFn)(NtpTimeBase* base, uint32_t* utc, uint32_t* boundaryUs);
typedef void (*NtpSentFn)(const NtpTimeBase &base);

class NtpServer
{
public:
    NtpServer(WiFiUDP Port)
    {
        timeServerPort_=Port;
    }

    void setSource(NtpSourceFn source, NtpSentFn sent)
    {
        source_ = source;
        sent_ = sent;
    }

    /*
     * Begins listening for NTP requests.
     */
    bool beginListening(void);

    void stopListening(void);   // Release port 123

    /*
     * Processes a single NTP request.
     * microsecs: micros() at the last second boundary (ESP8266 polled path; idle with a callback).
     */
    bool processOneRequest(uint32_t utc, uint32_t microsecs, const NtpTimeBase &base);

    void poll(void);                 // Polled path: answer or drop one request with the time from source_

    void dropRequest(void) {         // Polled: no time, so no late reply either
#ifndef ESP32
      timeServerPort_.parsePacket();
#endif
    }

    bool polled(void) const {
#ifdef ESP32
      return (nullptr == udp_.pcb);
#else
      return true;
#endif
    }

    NtpSourceFn source_ = nullptr;   // Public for NtpRecv()
    NtpSentFn sent_ = nullptr;
#ifdef ESP32
    NtpUdp udp_;                     // ESP32 socket
#endif

private:
    WiFiUDP timeServerPort_;
};

#endif // NTP_SERVER_H
