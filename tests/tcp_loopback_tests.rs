extern crate mavulator;

mod test_shared;


#[cfg(test)]
mod test_tcp_connections {
    use std::thread;
    use crate::test_shared;
    use uorb_codec::common::*;
    use uorb_codec::{UorbMsgMeta};

    /// Test whether we can send a message via TCP and receive it OK
    #[test]
    pub fn test_tcp_loopback() {
        const RECEIVE_CHECK_COUNT: i32 = 5;

        let server_thread = thread::spawn( {
            move || {
                let server = mavulator::connection::select_protocol("tcpin:0.0.0.0:14550")
                    .expect("Couldn't create server");

                let mut recv_count = 0;
                for _i in 0..RECEIVE_CHECK_COUNT {
                    match server.recv() {
                        Ok((_header, msg)) => {
                            match msg {
                                uorb_codec::common::UorbMessage::VehicleStatus(_vehicle_status_msg) => {
                                    recv_count += 1;

                                },
                                _ => {
                                    // one message parse failure fails the test
                                    break;
                                }
                            }
                        }
                        Err(..) => {
                            // one message read failure fails the test
                            break;
                        }
                    }
                }
                assert_eq!(recv_count, RECEIVE_CHECK_COUNT);
            }
        });

        // have the client send a few hearbeats
        thread::spawn({
            move || {
                let msg_data = test_shared::get_vehicle_status();
                let msg = uorb_codec::common::UorbMessage::VehicleStatus( msg_data.clone() );
                let header = uorb_codec::UorbHeader {
                    version: uorb_codec::UORB_MAGIC_V1,
                    hash: VehicleStatusData::MSG_HASH_CODE,
                    timestamp: 666,
                    instance_id: 17,
                    payload_len: VehicleStatusData::ENCODED_LEN,
                };
                let client = mavulator::connection::select_protocol("tcpout:127.0.0.1:14550")
                    .expect("Couldn't create client");
                for _i in 0..RECEIVE_CHECK_COUNT {
                    client.send(&header,&msg).ok();
                }
            }
        });

        server_thread.join().unwrap();
    }

}