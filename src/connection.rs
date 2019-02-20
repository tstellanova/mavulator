


use std::net::{TcpListener, TcpStream};
use std::time::Duration;
use std::sync::Mutex;
use std::net::{ToSocketAddrs};
use std::io::{self};

use uorb_codec::{UorbHeader, UorbMessage};
use uorb_codec::read_msg;
use uorb_codec::write_msg;

pub trait UorbConnection {

    /// Receive a message.
    ///
    /// Blocks until a valid frame is received, ignoring invalid messages.
    fn recv(&self) -> io::Result<(UorbHeader,UorbMessage)>;

    /// Send a  message
    fn send(&self, header: &UorbHeader, data: &UorbMessage) -> io::Result<()>;
}


/// TCP  connection

pub fn select_protocol(address: &str) -> io::Result<Box<UorbConnection + Sync + Send>> {
    if address.starts_with("tcpout:")  {
        Ok(Box::new(tcpout(&address["tcpout:".len()..])?))
    } else if address.starts_with("tcpin:") {
        Ok(Box::new(tcpin(&address["tcpin:".len()..])?))
    }
    else {
        Err(io::Error::new(io::ErrorKind::AddrNotAvailable,"Protocol unsupported"))
    }
}

pub fn tcpout<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let addr = address.to_socket_addrs().unwrap().next().expect("Host address lookup failed.");
    let socket = TcpStream::connect(&addr)?;
    socket.set_read_timeout(Some(Duration::from_millis(100)))?;

    Ok(TcpConnection {
        reader: Mutex::new(socket.try_clone()?),
        writer: Mutex::new(TcpWrite {
            socket: socket,
        }),
    })
}

pub fn tcpin<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let addr = address.to_socket_addrs().unwrap().next().expect("Invalid address");
    let listener = TcpListener::bind(&addr)?;

    //For now we only accept one incoming stream: this blocks until we get one
    for incoming in listener.incoming() {
        match incoming {
            Ok(socket) => {
                return Ok(TcpConnection {
                    reader: Mutex::new(socket.try_clone()?),
                    writer: Mutex::new(TcpWrite {
                        socket: socket,
                    }),
                })
            },
            Err(e) => {
                //println!("listener err: {}", e);
                return Err(e);
            },
        }
    }
    Err(io::Error::new(
        io::ErrorKind::NotConnected,
        "No incoming connections!",
    ))
}

pub struct TcpConnection {
    reader: Mutex<TcpStream>,
    writer: Mutex<TcpWrite>,
}

struct TcpWrite {
    socket: TcpStream,
}


impl UorbConnection for TcpConnection {
    fn recv(&self) -> io::Result<(UorbHeader, UorbMessage)> {
        let mut lock = self.reader.lock().expect("tcp read failure");
        read_msg(&mut *lock)
    }

    fn send(&self, header: &UorbHeader, data: &UorbMessage) -> io::Result<()> {
        let mut lock = self.writer.lock().unwrap();
        write_msg(&mut lock.socket, header, data)
    }

}
