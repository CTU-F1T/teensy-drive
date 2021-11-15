#ifndef TEENSY_DRIVE_SERIAL_PORT_HPP_
#define TEENSY_DRIVE_SERIAL_PORT_HPP_

#include <string>

#include <termios.h>

class SerialPort {

public:
	explicit SerialPort();

	void open(std::string device);

	void close();

	void get_modem_bits(int &bits) const;

	void set_modem_bits(const int &bits) const;

	void set_indicated_modem_bits(const int &bits) const;

	void clear_indicated_modem_bits(const int &bits) const;

	~SerialPort();

	int getFd() const;



private:
	std::string device_{};
	int fd_{-1};
	bool tty_restore_needed_{false};
	struct termios tty_{};
	struct termios original_tty_{};

};


#endif // TEENSY_DRIVE_SERIAL_PORT_HPP_
