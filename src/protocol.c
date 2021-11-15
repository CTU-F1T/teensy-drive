#include <stdlib.h>
#include "protocol.h"
#include "crc.h"
#include "unistd.h"

struct packet_handler_with_context {
	packet_handler handler;
	void *context;
};

static struct packet_handler_with_context packet_handlers[] = {
	[0] = {NULL, NULL},
	[MESSAGE_ESTOP] = {NULL, NULL},
	[MESSAGE_DRIVE_PWM] = {NULL, NULL},
	[MESSAGE_PWM_HIGH] = {NULL, NULL},
};

static int packet_type_to_payload_size(uint16_t type) {

	if (type >= sizeof(packet_type_to_payload_size_table)) {
		return -1;
	}

	return packet_type_to_payload_size_table[type];

}

/**
 * TODO: document
 */
void process_messages(const uint8_t *data, int size) {

	static const int payload_max_size = packet_max_size - 4;
	static union packet packet;
	static uint16_t *packet_type = ((uint16_t *) &packet);
	static uint8_t *buffer = (uint8_t *) &packet;
	static int buffer_size = 0;
	static int payload_size = 0;
	static int packet_size = 0;

	const uint8_t *data_end_ptr = data + size;
	const uint8_t *byte_ptr = data;

	while (byte_ptr < data_end_ptr) {

		uint8_t byte = *byte_ptr++;

		// save byte to buffer and then increase buffer index
		buffer[buffer_size++] = byte;

		// once we have packet type we validate it and use it to determine the packet payload size
		if (buffer_size == 2) {
			payload_size = packet_type_to_payload_size(*((uint16_t *) &packet));
			packet_size = 2 + payload_size + 2;
			if (payload_size < 1 || payload_size > payload_max_size) {
				// reset buffer index on invalid packet type (drops packet type)
				buffer_size = 0;
			}
			continue;
		}

		// packet is complete
		if (buffer_size == packet_size) {

			// validate checksum
			uint16_t checksum = crc16(buffer, buffer_size);

			// if we calculate CRC over the whole packet (including the checksum) it must equal to 0
			if (checksum != 0) {
				// broken packet
				// TODO: maybe log
				// reset buffer
				buffer_size = 0;
				continue;
			}

			struct packet_handler_with_context *reg = &packet_handlers[*packet_type];
			if (reg->handler != NULL) {
				reg->handler(&packet, reg->context);
			}

			// reset buffer
			buffer_size = 0;
			// do receive more than one packet
			continue;

		}

	}

}

/**
 * Sets the handler that is invoked every time a packet of the given type
 * is received in try_receive_packet()
 *
 * @param type the packet type
 * @param handler the handler function with one argument which is a pointer to the received packet,
 *                the packet must not be accessed upon returning from the handler
 */
void set_packet_handler(enum packet_type type, packet_handler handler, void *context) {
	struct packet_handler_with_context *reg = &packet_handlers[type];
	reg->handler = handler;
	reg->context = context;
}

/**
 * Send the given packet using write with the given file descriptor
 *
 * The packet type must be correctly set.
 * NOTE: This function mutates the packet. It updates the checksum field in the packet.
 *
 * @param packet the packet to send, its checksum is recalculated before sending
 * @param fd the file descriptor for use with write
 * @return true if the packet was sent successfully, false otherwise
 */
bool send_packet(int fd, union packet *packet) {

	uint16_t type = *((uint16_t *) packet); // first two bytes are packet type
	int payload_size = packet_type_to_payload_size(type);

	if (payload_size == -1) {
		return -1;
	}

	// two bytes checksum field is at the end of packet
	uint8_t *checksum_byte = ((uint8_t *) packet) + 2 + payload_size;

	// calculate CRC over the type and the payload
	uint16_t checksum = crc16((uint8_t *) packet, payload_size + 2);

	// write checksum in big endian order (MSB at the lower address)
	// so that we can validate it by calculating CRC over the whole packet and checking for 0
	*checksum_byte++ = checksum >> 8; // packet[size - 2]
	*checksum_byte = checksum & 0xFF; // packet[size - 1]

	// send the whole packet (including two byte type and two bytes checksum)
	int result = (int) write(fd, packet, payload_size + 4);

	return result == payload_size;

}
