/*
 * File: NTPServer.h
 * Description:
 *   NTP server implementation.
 * Author: Mooneer Salem <mooneer@gmail.com>
 * License: New BSD License
 */
 
#ifndef NTP_SERVER_H
#define NTP_SERVER_H

class NtpServer
{
public:
    NtpServer(WiFiUDP Port)
    {
        timeServerPort_=Port;
    }
    
    /*
     * Begins listening for NTP requests.
     */
    bool beginListening(void);
    
    
    /*
     * Processes a single NTP request.
     * millisecs must be a millis() timestamp of the last second boundary.
     * Set pps true when it comes from a GPS pulse per second edge rather than from the
     * arrival of a serial message, so the reply can advertise the accuracy we then actually
     * have instead of the original conservative estimates.
     */
    bool processOneRequest(uint32_t utc, uint32_t millisecs, bool pps = false);
    
private:
    WiFiUDP timeServerPort_;
};

#endif // NTP_SERVER_H
