package main

import (
	"github.com/mntmn/portmidi"
	"github.com/tarm/serial"
	"github.com/veandco/go-sdl2/sdl"
	"log"
	"net"
	"strconv"
	"strings"
	"time"
)

func readSerLine(s *serial.Port) string {
	buf := make([]byte, 1)
	line := ""

	for buf[0] != '\r' {
		n, err := s.Read(buf)
		if err != nil || n == 0 {
			//log.Fatal(err)
			return line
		}

		if n > 0 && buf[0] != '\r' {
			line += string(buf)
		}
	}

	//log.Println("line: ", line)
	return line
}

func writeSerLine(s *serial.Port) {
	s.Write([]byte("\n"))
}

func netSend(c net.Conn, data string) {
	body := []byte(data)
	hdr := []byte{0, 0, 0, byte(len(body))}
	msg := append(hdr[:], body[:]...)
	//fmt.Println("sending:", msg)
	c.Write(msg)
	//fmt.Println("sent: ", n, err)
}

func main() {
	sdl.Init(sdl.INIT_EVERYTHING)

	ww := 400
	wh := 300

	portmidi.Initialize()
	log.Println("midi devices: ", portmidi.CountDevices())
	for i := 0; i < portmidi.CountDevices(); i++ {
		log.Println("midi", i, portmidi.Info(portmidi.DeviceID(i)))
	}
	midiOut, err := portmidi.NewOutputStream(portmidi.DefaultOutputDeviceID(), 128, 0)
	if err != nil {
		log.Fatal(err)
	}
	defer midiOut.Close()

	window, err := sdl.CreateWindow("Musicsuit", sdl.WINDOWPOS_UNDEFINED, sdl.WINDOWPOS_UNDEFINED,
		ww, wh, sdl.WINDOW_SHOWN)
	if err != nil {
		panic(err)
	}
	defer window.Destroy()

	/*surface, err := window.GetSurface()
	if err != nil {
		panic(err)
	}*/

	c := &serial.Config{Name: "/dev/ttyACM0", Baud: 115200, ReadTimeout: time.Second * 1}
	s, err := serial.OpenPort(c)
	if err != nil {
		log.Fatal(err)
	}

	//writeSerLine(s)
	readSerLine(s)
	//note := 0

	for {
		sdl.PumpEvents()
		if _, _, mstate := sdl.GetMouseState(); mstate == 0 {
			//time.Sleep(5 * time.Millisecond)
			instr := strings.Replace(readSerLine(s), "\n", "", -1)
			sxyz := strings.Split(instr, ",")
			if len(sxyz) == 3 {
				x, _ := strconv.Atoi(sxyz[0])
				y, _ := strconv.Atoi(sxyz[1])
				z, _ := strconv.Atoi(sxyz[2])

				//log.Println(x, y, z)

				if x == 176 {
					// controlchange
					//rect := sdl.Rect{0, 0, int32(ww - 1), int32(wh - 1)}
					//surface.FillRect(&rect, uint32(z<<uint(note*8)))
					midiOut.WriteShort(int64(x), int64(y), int64(z))
				}

				if x == 144 && z != 0 {
					// note on
					//note = 1
					midiOut.WriteShort(int64(x), int64(y), int64(z))
				}
				if x == 144 && z == 0 {
					// note on
					//note = 0
					midiOut.WriteShort(int64(x), int64(y), int64(z))
				}

			} else {
				log.Println("in: ", instr)
			}

			window.UpdateSurface()
			sdl.Delay(1)
		} else {
			break
		}
	}
	sdl.Quit()
}
