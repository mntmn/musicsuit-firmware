package main

import (
	"github.com/go-gl/gl/v2.1/gl"
	"github.com/mntmn/portmidi"
	"github.com/tarm/serial"
	"github.com/veandco/go-sdl2/sdl"
	"log"
	"math"
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

var qx, qy, qz, qbx, qby, qbz float32
var context sdl.GLContext
var window *sdl.Window
var ww = 800
var wh = 600
var noteOn = 0

func main() {
	sdl.Init(sdl.INIT_EVERYTHING)

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

	if err := gl.Init(); err != nil {
		panic(err)
	}

	window, err = sdl.CreateWindow("Musicsuit", sdl.WINDOWPOS_UNDEFINED, sdl.WINDOWPOS_UNDEFINED,
		ww, wh, sdl.WINDOW_SHOWN|sdl.WINDOW_OPENGL)
	if err != nil {
		panic(err)
	}
	defer window.Destroy()

	/*surface, err := window.GetSurface()
	if err != nil {
		panic(err)
	}*/

	c := &serial.Config{Name: "/dev/ttyUSB0", Baud: 115200, ReadTimeout: time.Second * 1}
	s, err := serial.OpenPort(c)
	if err != nil {
		log.Fatal(err)
	}

	//writeSerLine(s)
	readSerLine(s)
	//note := 0

	go renderLoop()

	lastt := time.Now()
	yhist := make([]int, 4)
	lasty := float32(0)
	laston := time.Now()
	lastnote := int64(0)

	for {
		dt := time.Since(lastt)
		lastt = time.Now()

		sdl.PumpEvents()
		if _, _, mstate := sdl.GetMouseState(); mstate == 0 {
			//time.Sleep(5 * time.Millisecond)
			instr := strings.Replace(readSerLine(s), "\n", "", -1)
			sxyz := strings.Split(instr, "\t")
			if len(sxyz) == 5 {
				sensor, _ := strconv.Atoi(sxyz[0])

				x, _ := strconv.Atoi(sxyz[1])
				y, _ := strconv.Atoi(sxyz[2])
				z, _ := strconv.Atoi(sxyz[3])

				if sensor == 0 {
					qx = float32(x) / 100.0
					qy = float32(y) / 100.0
					qz = float32(z) / 100.0

					dy := math.Abs(10000 * float64(qy-lasty) / (float64(dt / 1000000)))

					lasty = qy
					//log.Println(qx, qy, qz)
					yhist = append(yhist[1:], int(dy))
					log.Println(yhist)

					lastonms := float64(time.Since(laston) / 1000000)

					/*if yhist[0] < yhist[1] && yhist[1] < yhist[2] && yhist[2] < yhist[3] {
						noteOn = 1
					} else {
						noteOn = 0
					}*/

					//p1 := math.Abs(float64(yhist[0]-yhist[1])) / 2
					//p2 := math.Abs(float64(yhist[3]-yhist[4])) / 2
					thr := 20

					if lastonms >= 400 {
						if yhist[1] > thr && yhist[3] > thr {
							if noteOn == 0 {
								if qx < 0.3 {
									lastnote = 0
								} else {
									lastnote = 1
								}
								velo := int64((yhist[1] + yhist[3]) / 2 * 2)
								log.Println("velo: ", velo)
								midiOut.WriteShort(144, 52+lastnote, velo)
								laston = time.Now()

							}
							noteOn = 1
						} else {
							if noteOn == 1 {
								midiOut.WriteShort(144, 52+lastnote, 0)
							}
							noteOn = 0
						}
					}
				} else {
					qbx = float32(x) / 100.0
					qby = float32(y) / 100.0
					qbz = float32(z) / 100.0
					//log.Println("s1: ", qx, qy)
					midiOut.WriteShort(0xb0+0, int64(sensor)*2, int64(64+qx*64))
					//midiOut.WriteShort(0xb0+0, sensor*2, qy)
				}
			} else {
				log.Println("in: ", instr)
			}

			//window.UpdateSurface()
		} else {
			break
		}
		//
	}
	sdl.Quit()
}

func renderLoop() {
	var err error
	if context, err = sdl.GL_CreateContext(window); err != nil {
		panic(err)
	}
	defer sdl.GL_DeleteContext(context)
	gl.Enable(gl.DEPTH_TEST)
	gl.ClearColor(0.2, 0.2, 0.3, 1.0)
	gl.ClearDepth(1)
	gl.DepthFunc(gl.LEQUAL)
	gl.Viewport(0, 0, int32(ww), int32(wh))
	for {
		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		gl.Begin(gl.TRIANGLES)
		if noteOn == 1 {
			gl.Color3f(1.0, 0.0, 1.0)
		} else {
			gl.Color3f(0.0, 0.0, 1.0)
		}
		gl.Vertex2f(0.5, 0.0)
		gl.Vertex2f(0.2, 0.0)
		gl.Vertex2f(0.35+qz, qy)

		gl.Color3f(0.0, 0.0, 1.0)
		gl.Vertex2f(-0.5, 0.0)
		gl.Vertex2f(-0.2, 0.0)
		gl.Vertex2f(-0.35+qbz, qby)
		gl.End()

		sdl.GL_SwapWindow(window)
		sdl.Delay(1)
	}
}
