#include "serial_port.hpp"
#include "utils.h"

#include <iostream>
#include <utility>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() {
	debug("SerialPort destructor");
	close();
}

void SerialPort::open(std::string device) {

	if (fd_ != -1) {
		throw std::runtime_error("cannot open while already opened");
	}

	debug("opening '" << device << "' ...");

	// O_NONBLOCK is needed to not wait for the CDC signal. See tty_ioctl(4).
	fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd_ < 0) {
		fd_ = -1;
		// TODO: format error string, add result and strerror(errno)
		throw std::runtime_error("failed to open");
	}

	device_ = std::move(device);

	// cancel the effect of O_NONBLOCK flag
	int n = fcntl(fd_, F_GETFL, 0);
	fcntl(fd_, F_SETFL, n & ~O_NDELAY);

	debug("opened " << device_ << " for reading and writing");

	int result;

	result = tcgetattr(fd_, &tty_);

	if (result < 0) {
		// TODO: format error string, add result and strerror(errno)
		throw std::runtime_error("failed to tcgetattr");
	}

	// preserve original settings for restoration
	original_tty_ = tty_;

	// baud rate
	speed_t spd = B57600;
	cfsetospeed(&tty_, spd);
	cfsetispeed(&tty_, spd);

	cfmakeraw(&tty_);

	tty_.c_cc[VMIN] = 1; // MIN = n bytes
	tty_.c_cc[VTIME] = 0; // TIME = n * 0.1 second
	// from man termios page:
	//   In noncanonical mode input is available immediately (without the user having to type a line-delimiter character), no input
	//   processing is performed, and line editing is disabled.  The read buffer will only accept 4095  chars;  this  provides  the
	//   necessary  space for a newline char if the input mode is switched to canonical.  The settings of MIN (c_cc[VMIN]) and TIME
	//   (c_cc[VTIME]) determine the circumstances in which a read(2) completes; there are four distinct cases:
	//
	//   MIN == 0, TIME == 0 (polling read)
	//          If data is available, read(2) returns immediately, with the lesser of the number of bytes available, or the  number
	//          of bytes requested.  If no data is available, read(2) returns 0.
	//
	//   MIN > 0, TIME == 0 (blocking read)
	//          read(2) blocks until MIN bytes are available, and returns up to the number of bytes requested.
	//
	//   MIN == 0, TIME > 0 (read with timeout)
	//          TIME  specifies the limit for a timer in tenths of a second.  The timer is started when read(2) is called.  read(2)
	//          returns either when at least one byte of data is available, or when the timer expires.  If the timer expires  with‐
	//          out  any  input  becoming  available,  read(2)  returns 0.  If data is already available at the time of the call to
	//          read(2), the call behaves as though the data was received immediately after the call.
	//
	//   MIN > 0, TIME > 0 (read with interbyte timeout)
	//          TIME specifies the limit for a timer in tenths of a second.  Once an initial byte of input becomes  available,  the
	//          timer  is  restarted  after each further byte is received.  read(2) returns when any of the following conditions is
	//          met:
	//
	//          *  MIN bytes have been received.
	//
	//          *  The interbyte timer expires.
	//
	//          *  The number of bytes requested by read(2) has been received.  (POSIX does not specify this termination condition,
	//             and on some other implementations read(2) does not return in this case.)
	//
	//          Because  the  timer  is  started only after the initial byte becomes available, at least one byte will be read.  If
	//          data is already available at the time of the call to read(2), the call behaves as though the data was received  im‐
	//          mediately after the call.
	//
	//   POSIX  does not specify whether the setting of the O_NONBLOCK file status flag takes precedence over the MIN and TIME set‐
	//   tings.  If O_NONBLOCK is set, a read(2) in noncanonical mode may return immediately, regardless of the setting of  MIN  or
	//   TIME.   Furthermore,  if no data is available, POSIX permits a read(2) in noncanonical mode to return either 0, or -1 with
	//   errno set to EAGAIN.

	// ensure 1 stop-bit only
	tty_.c_cflag &= ~CSTOPB;
	// ensure no HW flow control
	tty_.c_cflag &= ~CRTSCTS;
	tty_.c_cflag |= CLOCAL | CREAD;

	result = tcsetattr(fd_, TCSANOW, &tty_);

	if (result < 0) {
		// TODO: format error string, add result and strerror(errno)
		throw std::runtime_error("failed to tcsetattr");
	}

	tty_restore_needed_ = true;

}

void SerialPort::close() {

	debug("close invoked");

	if (fd_ == -1) {
		return;
	}

	debug("closing ...");

	if (tty_restore_needed_) {
		// it may fail, but we do not care ...
		tcsetattr(fd_, TCSANOW, &original_tty_);
		tty_restore_needed_ = false;
	}

	::close(fd_);
	fd_ = -1;

	debug("closed");

}

void SerialPort::get_modem_bits(int &bits) const {
	// TIOCMGET = get the status of modem bits
	// see https://linux.die.net/man/4/tty_ioctl
	ioctl(fd_, TIOCMGET, bits);
}

void SerialPort::set_modem_bits(const int &bits) const {
	// TIOCMSET = set the status of modem bits
	// see https://linux.die.net/man/4/tty_ioctl
	ioctl(fd_, TIOCMSET, bits);
}

void SerialPort::set_indicated_modem_bits(const int &bits) const {
	// TIOCMBIS = set the indicated modem bits
	// see https://linux.die.net/man/4/tty_ioctl
	ioctl(fd_, TIOCMBIS, bits);
}

void SerialPort::clear_indicated_modem_bits(const int &bits) const {
	// TIOCMBIC = clear the indicated modem bits
	// see https://linux.die.net/man/4/tty_ioctl
	ioctl(fd_, TIOCMBIC, &bits);
}

int SerialPort::getFd() const {
	return fd_;
}
