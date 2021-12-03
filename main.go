package main

/*
#cgo LDFLAGS: -lpigpio -lm

#include "LoRa.h"

*/
import "C"

import (
    "fmt"
    "time"
)

//export RX
func RX(rx *C.rxData) {
    fmt.Println(C.GoString(rx.buf))
}
    

func main() {

    fmt.Println("start Lora")

    C.initLora()
    
    for{
        time.Sleep(1 * time.Second)

    }
}
