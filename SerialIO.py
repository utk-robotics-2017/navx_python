import time
from IMUProtocol import IMUProtocol
from AHRSProtocol import AHRSProtocol
from Structs.AHRSUpdate import AHRSUpdate
from Structs.AHRSPosUpdate import AHRSPosUpdate
from Structs.BoardID import BoardID
from Structs.BoardState import BoardState
from Structs.YPRUpdate import YPRUpdate
from Structs.GyroUpdate import GyroUpdate


class SerialIO:
    def __init__(self, ahrs):
        self.ahrs = ahrs
        self.ypr_update_data = YPRUpdate()
        self.gyro_update_data = GyroUpdate()
        self.ahrs_update_data = AHRSUpdate()
        self.ahrspos_update_data = AHRSPosUpdate()
        self.board_id = BoardID()
        self.board_state = BoardState()

    def enqueueIntegrationControlMessage(self, action):
        self.next_integration_control_action = action
        self.signal_transmit_integration_control = True

    def dispatchStreamResponse(self, response):
        self.board_state.cal_status = (response.flags & IMUProtocol.NAV6_FLAG_MASK_CALIBRATION_STATE)
        self.board_state.capability_flags = (response.flags & ~IMUProtocol.NAV6_FLAG_MASK_CALIBRATION_STATE)
        self.board_state.op_status = 0x04  # TODO:  Create a symbol for this
        self.board_state.selftest_status = 0x07  # TODO:  Create a symbol for this
        self.board_state.accel_fsr_g = response.accel_fsr_g
        self.board_state.gyro_fsr_dps = response.gyro_fsr_dps
        self.board_state.update_rate_hz = response.update_rate_hz
        self.ahrs.setBoardState(self.board_state)
        '''
            If AHRSPOS is update type is request, but board doesn't support it,
            retransmit the stream config, falling back to AHRS Update mode.
        '''
        if (self.update_type == AHRSProtocol.MSGID_AHRSPOS_UPDATE):
            if (not self.board_capabilities.isDisplacementSupported()):
                self.update_type = AHRSProtocol.MSGID_AHRS_UPDATE
                self.signal_retransmit_stream_config = True

    def decodePacketHandler(self, received_data, offset, bytes_remaining):
        sensor_timestamp = 0  # Note:  Serial Protocols don't provide sensor timestamps

        packet_length = IMUProtocol.decodeYPRUpdate(received_data, offset, bytes_remaining, self.ypr_update_data)
        if (packet_length > 0):
            self.ahrs.setYawPitchRoll(self.ypr_update_data, sensor_timestamp)
            return packet_length

        packet_length = AHRSProtocol.decodeAHRSPosUpdate(received_data, offset, bytes_remaining, self.ahrspos_update_data)
        if(packet_length > 0):
            self.ahrs.setAHRSPosData(self.ahrspos_update_data, sensor_timestamp)
            return packet_length

        packet_length = AHRSProtocol.decodeAHRSUpdate(received_data, offset, bytes_remaining, self.ahrs_update_data)
        if(packet_length > 0):
            self.ahrs.setAHRSData(self.ahrs_update_data, sensor_timestamp)
            return packet_length

        packet_length = IMUProtocol.decodeGyroUpdate(received_data, offset, bytes_remaining, self.gyro_update_data)
        if(packet_length > 0):
            self.ahrs.setRawData(self.gyro_update_data, sensor_timestamp)
            return packet_length

        packet_length = AHRSProtocol.decodeBoardIDGetResponse(received_data, offset, bytes_remaining, self.board_id)
        if(packet_length > 0):
            self.ahrs.setBoardID(self.board_id)
            return packet_length

        return 0

    def run(self):
        self.stop = False
        stream_response_received = False
        last_stream_command_sent_timestamp = 0.0
        last_data_received_timestamp = 0.0
        last_second_start_time = 0.0

        partial_binary_packet_count = 0
        stream_response_receive_count = 0
        timeout_count = 0
        discarded_bytes_count = 0
        port_reset_count = 0
        updates_in_last_second = 0
        integration_response_receive_count = 0

        try {
            serial_port.setReadBufferSize(256);
            serial_port.setTimeout(1.0);
            serial_port.enableTermination('\n');
            serial_port.flush();
            serial_port.reset();
        } catch (RuntimeException ex) {
            ex.printStackTrace();
        }

        stream_command = [0 for i in range(256)]
        integration_control_command = [0 for i in range(256)]
        response = StreamResponse()
        integration_control = IntegrationControl()
        integration_control_response = IntegrationControl()

        cmd_packet_length = IMUProtocol.encodeStreamCommand(stream_command, update_type, update_rate_hz)


        try {
            serial_port.reset();
            serial_port.write( stream_command, cmd_packet_length );
            cmd_packet_length = AHRSProtocol.encodeDataGetRequest( stream_command,  AHRSProtocol.AHRS_DATA_TYPE.BOARD_IDENTITY, (byte)0 ); 
            serial_port.write( stream_command, cmd_packet_length );
            serial_port.flush();
            port_reset_count++;
            if ( debug ) {
                SmartDashboard.putNumber("navX Port Resets", (double)port_reset_count);
            }
            last_stream_command_sent_timestamp = Timer.getFPGATimestamp();
        } catch (RuntimeException ex) {
            ex.printStackTrace();
        }


        remainder_bytes = 0
        byte[] remainder_data = null;

        while (not stop):
            try {

                # Wait, with delays to conserve CPU resources, until bytes have arrived.

                if (signal_transmit_integration_control):
                    integration_control.action = next_integration_control_action
                    signal_transmit_integration_control = False
                    next_integration_control_action = 0
                    cmd_packet_length = AHRSProtocol.encodeIntegrationControlCmd(integration_control_command, integration_control)
                    try {
                        serial_port.write( integration_control_command, cmd_packet_length );
                    } catch (RuntimeException ex2) {
                        ex2.printStackTrace();
                    }              

                if (not stop and (remainder_bytes == 0) and (serial_port.getBytesReceived() < 1)):
                    Timer.delay(1.0/update_rate_hz)

                packets_received = 0
                received_data = serial_port.read(256);
                bytes_read = len(received_data)
                byte_count += bytes_read

                '''
                    If a partial packet remains from last iteration, place that at
                    the start of the data buffer, and append any new data available
                    at the serial port.
                '''

                if ( remainder_bytes > 0 ) {
                    byte[] resized_array = new byte[remainder_bytes + bytes_read];
                    System.arraycopy(remainder_data, 0, resized_array, 0, remainder_bytes);
                    System.arraycopy(received_data, 0, resized_array, remainder_bytes, bytes_read);
                    received_data = resized_array;
                    bytes_read += remainder_bytes;
                    remainder_bytes = 0;
                    remainder_data = null;
                }

                if (bytes_read > 0) {
                    last_data_received_timestamp = Timer.getFPGATimestamp();
                    int i = 0;
                    // Scan the buffer looking for valid packets
                    while (i < bytes_read) {

                        // Attempt to decode a packet

                        int bytes_remaining = bytes_read - i;

                        if ( received_data[i] != IMUProtocol.PACKET_START_CHAR ) {
                            /* Skip over received bytes until a packet start is detected. */
                            i++;
                            discarded_bytes_count++;
                            if ( debug ) {
                                SmartDashboard.putNumber("navX Discarded Bytes", (double)discarded_bytes_count);
                            }
                            continue;
                        } else {
                            if ( ( bytes_remaining > 2 ) && 
                                    ( received_data[i+1] == AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR ) ) {
                                /* Binary packet received; next byte is packet length-2 */
                                byte total_expected_binary_data_bytes = received_data[i+2];
                                total_expected_binary_data_bytes += 2;
                                while ( bytes_remaining < total_expected_binary_data_bytes ) {

                                    /* This binary packet contains an embedded     */
                                    /* end-of-line character.  Continue to receive */
                                    /* more data until entire packet is received.  */
                                    byte[] additional_received_data = serial_port.read(256);
                                    byte_count += additional_received_data.length;
                                    bytes_remaining += additional_received_data.length;

                                    /* Resize array to hold existing and new data */
                                    byte[] c = new byte[received_data.length + additional_received_data.length];
                                    if ( c.length > 0 ) {
                                        System.arraycopy(received_data, 0, c, 0, received_data.length);
                                        System.arraycopy(additional_received_data, 0, c, received_data.length, additional_received_data.length);
                                        received_data = c;
                                    } else {
                                        /* Timeout waiting for remainder of binary packet */
                                        i++;
                                        bytes_remaining--;
                                        partial_binary_packet_count++;
                                        if ( debug ) {
                                            SmartDashboard.putNumber("navX Partial Binary Packets", (double)partial_binary_packet_count);
                                        }
                                        continue;
                                    }
                                }
                            }
                        }

                        int packet_length = decodePacketHandler(received_data,i,bytes_remaining);
                        if (packet_length > 0) {
                            packets_received++;
                            update_count++;
                            last_valid_packet_time = Timer.getFPGATimestamp();
                            updates_in_last_second++;
                            if ((last_valid_packet_time - last_second_start_time ) > 1.0 ) {
                                if ( debug ) {
                                    SmartDashboard.putNumber("navX Updates Per Sec", (double)updates_in_last_second);
                                }
                                updates_in_last_second = 0;
                                last_second_start_time = last_valid_packet_time;
                            }
                            i += packet_length;
                        } 
                        else 
                        {
                            packet_length = IMUProtocol.decodeStreamResponse(received_data, i, bytes_remaining, response);
                            if (packet_length > 0) {
                                packets_received++;
                                dispatchStreamResponse(response);
                                stream_response_received = true;
                                i += packet_length;
                                stream_response_receive_count++;
                                if ( debug ) {
                                    SmartDashboard.putNumber("navX Stream Responses", (double)stream_response_receive_count);                                
                                }
                            }
                            else {
                                packet_length = AHRSProtocol.decodeIntegrationControlResponse( received_data, i, bytes_remaining,
                                        integration_control_response );
                                if ( packet_length > 0 ) {
                                    // Confirmation of integration control
                                    integration_response_receive_count++;
                                    if ( debug ) {
                                        SmartDashboard.putNumber("navX Integration Control Response Count", integration_response_receive_count);
                                    }
                                    i += packet_length;
                                } else {
                                    /* Even though a start-of-packet indicator was found, the  */
                                    /* current index is not the start of a packet if interest. */
                                    /* Scan to the beginning of the next packet,               */
                                    boolean next_packet_start_found = false;
                                    int x;
                                    for ( x = 0; x < bytes_remaining; x++ ) {
                                        if ( received_data[i + x] != IMUProtocol.PACKET_START_CHAR) {
                                            x++;
                                        } else {
                                            i += x;
                                            bytes_remaining -= x;
                                            if ( x != 0 ) {
                                                next_packet_start_found = true;
                                            }
                                            break;
                                        }
                                    }
                                    boolean discard_remainder = false;
                                    if ( !next_packet_start_found && x == bytes_remaining ) {
                                        /* Remaining bytes don't include a start-of-packet */
                                        discard_remainder = true;
                                    }
                                    boolean partial_packet = false;
                                    if ( discard_remainder ) {
                                        /* Discard the remainder */
                                        i = bytes_remaining;
                                    } else {                                    
                                        if ( !next_packet_start_found ) {
                                            /* This occurs when packets are received that are not decoded.   */
                                            /* Bump over this packet and prepare for the next.               */
                                            if ( ( bytes_remaining > 2 ) && 
                                                    ( received_data[i+1] == AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR ) ) {
                                                /* Binary packet received; next byte is packet length-2 */
                                                int pkt_len = received_data[i+2];
                                                pkt_len += 2;
                                                if ( bytes_remaining >= pkt_len ) {
                                                    bytes_remaining -= pkt_len;
                                                    i += pkt_len;
                                                    discarded_bytes_count += pkt_len;
                                                    if ( debug ) {
                                                        SmartDashboard.putNumber("navX Discarded Bytes", (double)discarded_bytes_count);
                                                    }
                                                } else {
                                                    /* This is the initial portion of a partial binary packet. */
                                                    /* Keep this data and attempt to acquire the remainder.    */
                                                    partial_packet = true;
                                                }
                                            } else {
                                                /* Ascii packet received. */
                                                /* Scan up to and including next end-of-packet character       */
                                                /* sequence, or the beginning of a new packet.                 */
                                                for ( x = 0; x < bytes_remaining; x++ ) {
                                                    if ( received_data[i+x] == (byte)'\r') {
                                                        i += x+1;
                                                        bytes_remaining -= (x+1);
                                                        discarded_bytes_count += x+1;
                                                        if ( ( bytes_remaining > 0 ) &&  received_data[i] == (byte)'\n') {
                                                            bytes_remaining--;
                                                            i++;
                                                            discarded_bytes_count++;
                                                        }
                                                        if ( debug ) {
                                                            SmartDashboard.putNumber("navX Discarded Bytes", (double)discarded_bytes_count);
                                                        }
                                                        break;
                                                    }
                                                    /* If a new start-of-packet is found, discard */
                                                    /* the ascii packet bytes that precede it.    */
                                                    if ( received_data[i+x] == (byte)'!') {
                                                        if ( x > 0 ) {
                                                            i += x;
                                                            bytes_remaining -= x;
                                                            discarded_bytes_count += x;
                                                            break;
                                                        } else {
                                                            /* start of packet found, but no termination     */
                                                            /* Time to get some more data, unless the bytes  */
                                                            /* remaining are larger than a valid packet size */
                                                            if ( bytes_remaining < IMUProtocol.IMU_PROTOCOL_MAX_MESSAGE_LENGTH ) {
                                                                /* Get more data */
                                                                partial_packet = true;
                                                            } else {
                                                                i++;
                                                                bytes_remaining--;
                                                            }
                                                            break;
                                                        }
                                                    }
                                                }
                                                if ( x == bytes_remaining ) {
                                                    /* Partial ascii packet - keep the remainder */
                                                    partial_packet = true;
                                                }
                                            }
                                        }
                                    }
                                    if ( partial_packet ) {
                                        remainder_data = new byte[bytes_remaining];
                                        System.arraycopy(received_data,i,remainder_data,0,bytes_remaining);
                                        remainder_bytes = bytes_remaining;
                                        i = bytes_read;
                                    }
                                }
                            }
                        }
                    }

                    if ( ( packets_received == 0 ) && ( bytes_read == 256 ) ) {
                        // Workaround for issue found in SerialPort implementation:
                        // No packets received and 256 bytes received; this
                        // condition occurs in the SerialPort.  In this case,
                        // reset the serial port.
                        serial_port.flush();
                        serial_port.reset();
                        port_reset_count++;                        
                        if ( debug ) {
                            SmartDashboard.putNumber("navX Port Resets", (double)port_reset_count);
                        }
                    }

                    boolean retransmit_stream_config = false;
                    if ( signal_retransmit_stream_config ) {
                        retransmit_stream_config = true;
                        signal_retransmit_stream_config = false;
                    }

                    // If a stream configuration response has not been received within three seconds
                    // of operation, (re)send a stream configuration request

                    if ( retransmit_stream_config ||
                            (!stream_response_received && ((Timer.getFPGATimestamp() - last_stream_command_sent_timestamp ) > 3.0 ) ) ) {
                        cmd_packet_length = IMUProtocol.encodeStreamCommand( stream_command, update_type, update_rate_hz ); 
                        try {
                            resetSerialPort();
                            last_stream_command_sent_timestamp = Timer.getFPGATimestamp();
                            serial_port.write( stream_command, cmd_packet_length );
                            cmd_packet_length = AHRSProtocol.encodeDataGetRequest( stream_command,  AHRSProtocol.AHRS_DATA_TYPE.BOARD_IDENTITY, (byte)0 ); 
                            serial_port.write( stream_command, cmd_packet_length );
                            serial_port.flush();
                        } catch (RuntimeException ex2) {
                            ex2.printStackTrace();
                        }                                                    
                    }
                    else {                        
                        // If no bytes remain in the buffer, and not awaiting a response, sleep a bit
                        if ( stream_response_received && ( serial_port.getBytesReceived() == 0 ) ) {
                            Timer.delay(1.0/update_rate_hz);
                        }        
                    }

                    /* If receiving data, but no valid packets have been received in the last second */
                    /* the navX MXP may have been reset, but no exception has been detected.         */
                    /* In this case , trigger transmission of a new stream_command, to ensure the    */
                    /* streaming packet type is configured correctly.                                */

                    if ( ( Timer.getFPGATimestamp() - last_valid_packet_time ) > 1.0 ) {
                        last_stream_command_sent_timestamp = 0.0;
                        stream_response_received = false;
                    }
                } else {
                    /* No data received this time around */
                    if ( Timer.getFPGATimestamp() - last_data_received_timestamp  > 1.0 ) {
                        resetSerialPort();
                    }                   
                }
            } catch (RuntimeException ex) {
                // This exception typically indicates a Timeout, but can also be a buffer overrun error.
                stream_response_received = false;
                timeout_count++;
                if ( debug ) {
                    SmartDashboard.putNumber("navX Serial Port Timeout / Buffer Overrun", (double)timeout_count);
                    SmartDashboard.putString("navX Last Exception", ex.getMessage() + "; " + ex.toString());
                }
                ex.printStackTrace();
                resetSerialPort();
            }
        }
    }

    '''
        Indicates whether the navX MXP is currently connected
        to the host computer.  A connection is considered established
        whenever a value update packet has been received from the
        navX MXP within the last second.
        :return Returns true if a valid update has been received within the last second.
    '''
    def isConnected(self):
        time_since_last_update = time.time() - self.last_valid_packet_time
        return time_since_last_update <= 1.0

    '''
        Returns the count in bytes of data received from the
        navX MXP.  This could can be useful for diagnosing
        connectivity issues.

        If the byte count is increasing, but the update count
        (see getUpdateCount()) is not, this indicates a software
        misconfiguration.
        :return The number of bytes received from the navX MXP.
    '''
    def getByteCount(self):
        return self.byte_count

    '''
        Returns the count of valid update packets which have
        been received from the navX MXP.  This count should increase
        at the same rate indicated by the configured update rate.
        :return The number of valid updates received from the navX MXP.
    '''
    def getUpdateCount(self):
        return self.update_count

    def setUpdateRateHz(self, update_rate):
        self.update_rate_hz = update_rate

    def zeroYaw(self):
        self.enqueueIntegrationControlMessage(AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_YAW)

    def zeroDisplacement(self):
        self.enqueueIntegrationControlMessage((AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_DISP_X | AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_DISP_Y | AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_DISP_Z))

    def stop(self):
        self.stop = True
