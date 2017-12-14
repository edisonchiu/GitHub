
/**
 * 使用這個文件來定義自訂的函式和積木。
 * 進一步了解：https://makecode.microbit.org/blocks/custom
 */

/*
enum MyEnum {
    //% block="one"
    One,
    //% block="two"
    Two
}
*/
enum MAX7219_REG {
    //% block="one"
    //% block="two"
    NOOP = 0x00,
    DIGIT0 = 0x01,
    DIGIT1 = 0x02,
    DIGIT2 = 0x03,
    DIGIT3 = 0x04,
    DIGIT4 = 0x05,
    DIGIT5 = 0x06,
    DIGIT6 = 0x07,
    DIGIT7 = 0x08,
    DECODE_MODE = 0x09,
    INTENSITY = 0x0A,
    SCAN_LIMIT = 0x0B,
    SHUTDOWN = 0x0C,
    DISPLAY_TEST = 0x0F,
}

/**
 * 自訂的積木
 */
//% weight=100 color=#0fbc11 icon=""
namespace LedMatrix {

    //let debug = true

    let currentMAX7219: MAX7219;
    //let currentMAX7219: MAX7219 = null;

    /**
     * TODO: 在此描述函式
     * @param n 在此描述參數, eg: 5
     * @param s 在此描述參數, eg: "Hello"
     * @param e 在此描述參數
     */
    //% block
    export function setup(
        _pin_dta: DigitalPin,
        _pin_clk: DigitalPin,
        _pin_cs: DigitalPin,
        _device_num: number = 1): void {

        // Add code here
        //if (debug)
        //    serial.writeValue("LedMatrix.setup() begin", 0)

        if (currentMAX7219 != undefined) {
            currentMAX7219 = new MAX7219(_pin_dta, _pin_clk, _pin_cs, _device_num);
            currentMAX7219.init()

            //if (debug)
            //    serial.writeValue("LedMatrix.setup() ok", 0)
        }
    }

    export function setIntensity(value: number = 0x07) {
        if (currentMAX7219 == undefined)
            return
        currentMAX7219.setIntensity(value)
    }

    export function writeData(x: number, y: number, buffer: number[], width: number = 0) {
        if (currentMAX7219 == undefined)
            return
        currentMAX7219.writeData(x, y, buffer, width)
    }

    /**
     * class MAX7219
     * @param n 在此描述參數, eg: 5
     * @param s 在此描述參數, eg: "Hello"
     * @param e 在此描述參數
     */

    export class MAX7219 {

        pin_dta: number;
        pin_clk: number;
        pin_cs: number;
        maxDevices: number;
        dir_mode: number;
        //buffer: number[];
        font_buffer_unit_size: number;

        constructor(
            //Setup(
            _pin_dta: DigitalPin = DigitalPin.P15,
            _pin_clk: DigitalPin = DigitalPin.P13,
            _pin_cs: DigitalPin = DigitalPin.P16,
            _max_devices: number) {
            //if (debug) {
            //    serial.writeValue("LedMatrix.MAX7219.constructor()", 0)
            //    serial.writeValue("_pin_dta", _pin_dta)
            //    serial.writeValue("_pin_clk", _pin_clk)
            //    serial.writeValue("_pin_cs", _pin_cs)
            //    serial.writeValue("_max_devices", _max_devices)
            //}

            // Const enum DigitalPin
            this.pin_dta = _pin_dta;
            this.pin_clk = _pin_clk;
            this.pin_cs = _pin_cs;

            this.maxDevices = _max_devices;
            this.dir_mode = 0;
            //this.buffer.length = 80;
            this.font_buffer_unit_size = 7;
        }

        init() {
            //if (debug)
            //    serial.writeValue("LedMatrix.MAX7219.init()", 0)

            pins.digitalWritePin(this.pin_dta, 1)
            pins.digitalWritePin(this.pin_clk, 1)
            pins.digitalWritePin(this.pin_cs, 1)

            this.setCommand(MAX7219_REG.SCAN_LIMIT, 0x07);
            this.setCommand(MAX7219_REG.DECODE_MODE, 0x00);  // using an led matrix (not digits)
            this.setCommand(MAX7219_REG.SHUTDOWN, 0x01);    // not in shutdown mode
            this.setCommand(MAX7219_REG.DISPLAY_TEST, 0x00); // no display test

            // empty registers, turn all LEDs off
            this.clearDisplay();

            this.setIntensity(0x0f);    // the first 0x0f is the value you can set
        }

        getDeviceCount(): number {
            return this.maxDevices;
        }

        writeData(x: number, y: number, buffer: number[], width: number = 0) {
            let c = 0
            let r = 0
            let w = buffer[0];
            let h = buffer[1];

            //if (debug) {
            //    serial.writeValue("LedMatrix.MAX7219.writeData()", 0)
            //    serial.writeValue("w", w)
            //    serial.writeValue("h", h)
            //    serial.writeValue("val[0]", buffer[2])
            //}

            if (width != 0)
                w = width

            if (h == 8 && y == 0) {
                for (let i = 0; i < w; i++) {
                    if ((this.dir_mode & 0x01) != 0) {
                        c = x + (7 - i)
                    } else {
                        c = x + i
                    }
                    if (c >= 0 && c < 80)
                        this.setColumn(c, buffer[i + 2])
                }
            } else {
                /*
                for (let i = 0; i < w; i++) 
                    for (let j= 0; j < h; j++) {
                        c = x + i;
                        r = y + j;
                        if (c >= 0 && c < 80 && r >= 0 && r < 8)
                            setDot(c, r, bitRead(buffer[i + 2], j));
                }
                */
            }
        }

        clearDisplay() {
            for (let i = 0; i < 8; i++)
                this.setColumn(i, 0)
        }

        setColumn(col: number, data: number) {
            let n = col / 8
            let c = col % 8
            pins.digitalWritePin(this.pin_cs, 0)
            for (let i = 0; i < this.maxDevices; i++) {
                if (i == n) {
                    this.setCommand(c + 1, data)
                } else {
                    this.setCommand(0, 0)
                }
            }
            pins.digitalWritePin(this.pin_cs, 0)
            pins.digitalWritePin(this.pin_cs, 1)
        }

        setCommand(command: number, value: number) {
            // set CS_PIN = LOW
            pins.digitalWritePin(this.pin_cs, 0)
            for (let i = 0; i < this.maxDevices; i++) {
                this.write_byte(command)
                this.write_byte(value)
            }
            // set CS_PIN = HIGH
            pins.digitalWritePin(this.pin_cs, 0)
            pins.digitalWritePin(this.pin_cs, 1)
        }

        write_byte(value: number) {
            for (let i = 0; i < 8; i++) {
                pins.digitalWritePin(this.pin_clk, 0) // CLOCK_PIN = LOW
                pins.digitalWritePin(this.pin_dta, ((value << i) & 0x80) >> 7) // DATA_PIN
                pins.digitalWritePin(this.pin_clk, 1) // CLOCK_PIN = HIGH
            }
            pins.digitalWritePin(this.pin_dta, 1)
        }

        // cmd: 0x09, value : 0x0~
        setDecodeMode(value: number) {
            this.setCommand(MAX7219_REG.DECODE_MODE, value)
        }
        // cmd: 0x0A, value : 0x0~0xF
        setIntensity(value: number) {
            this.setCommand(MAX7219_REG.INTENSITY, value);
        }
        // cmd: 0x0B, value : 0x0~0x7
        setScanLimit(value: number) {
            this.setCommand(MAX7219_REG.SCAN_LIMIT, value)
        }
        // cmd: 0x0C, value : 0x0~0x01
        setShutdownMode(value: number) {
            this.setCommand(MAX7219_REG.SHUTDOWN, value)
        }
        // cmd: 0x0F, value : 0x0~0x01
        setDisplayTest(value: number) {
            this.setCommand(MAX7219_REG.DISPLAY_TEST, value)
        }

        // max7219_set_dir() value -> set and 0x01 : vertical
        // invert -> set and 0x02 : horizontal invert
        setDir(value: number) {
            this.dir_mode = value
        }
    }


    /**
     * TODO: 在此描述函式
     * @param n 在此描述參數, eg: 5
     * @param s 在此描述參數, eg: "Hello"
     * @param e 在此描述參數
     */
    //% block
    /*
    export function foo(n: number, s: string, e: MyEnum): void {
        // Add code here
    }
    */
    /**
     * TODO: 在此描述函式
     * @param value 在此描述值, eg: 5
     */
    //% block
    /*
    export function fib(value: number): number {
        return value <= 1 ? value : fib(value - 1) + fib(value - 2);
    }
    */
}
