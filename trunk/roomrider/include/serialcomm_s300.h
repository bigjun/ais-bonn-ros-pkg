/*
 * serialcomm_s300.h
 *
 * written by: ?
 * modified by Andreas Hochrath & Torsten Fiolka in August 2010
 *
 * at   University Bonn,
 *              Institute for Computer Science VI,
 *              Autonomous Intelligent Systems
 *
 * contact: fiolka@cs.uni-bonn.de, hochrath@cs.uni-bonn.de
 *
 * We have no idea who wrote this driver originally, since no header was provided.
 * If you know who wrote this driver, please let us know so we can complete the header.
 *
 */


#ifndef __SERIALCOMMS300_H__
#define __SERIALCOMMS300_H__

#include <string>

#define RX_BUFFER_SIZE 4096
#define DEFAULT_SERIAL_PORT "/dev/sick300"
#define DEFAULT_BAUD_RATE 500000


class SerialCommS300 {
public:

	SerialCommS300();
	~SerialCommS300();

	// returns 0 if new laser data has arrived
	int readData();

	inline unsigned int getNumRanges() { return m_rangesCount; }
	inline float* getRanges() { return m_ranges; }

	int connect( const std::string& deviceName, unsigned int baudRate = DEFAULT_BAUD_RATE );
	int disconnect();

private:

	void setFlags();

	int setBaudRate( int baudRate );
	int baudRateToBaudCode( int baudCode );

	unsigned short createCRC( unsigned char* data, ssize_t len );


protected:

	unsigned char m_rxBuffer[ RX_BUFFER_SIZE ];

	int m_fd;

	int m_rxCount;

	float* m_ranges;
	unsigned int m_rangesCount;

};


#endif // __SERIALCOMMS300_H__



