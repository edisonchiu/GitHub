let p = 0
function font_init()  {
    buffer = [1, 2, 4, 8, 16, 32, 64, 128]
    font = [[3, 8, 0, 0, 0, 0, 0], [1, 8, 95, 0, 0, 0, 0], [3, 8, 3, 0, 3, 0, 0], [5, 8, 20, 62, 20, 62, 20], [4, 8, 36, 106, 43, 18, 0], [5, 8, 99, 19, 8, 100, 99], [5, 8, 54, 73, 86, 32, 80], [1, 8, 3, 0, 0, 0, 0], [3, 8, 28, 34, 65, 0, 0], [3, 8, 65, 34, 28, 0, 0], [5, 8, 40, 24, 14, 24, 40], [5, 8, 8, 8, 62, 8, 8], [2, 8, 176, 112, 0, 0, 0], [4, 8, 8, 8, 8, 8, 0], [2, 8, 96, 96, 0, 0, 0], [4, 8, 96, 24, 6, 1, 0], [4, 8, 62, 65, 65, 62, 0], [3, 8, 66, 127, 64, 0, 0], [4, 8, 98, 81, 73, 70, 0], [4, 8, 34, 65, 73, 54, 0], [4, 8, 24, 20, 18, 127, 0], [4, 8, 39, 69, 69, 57, 0], [4, 8, 62, 73, 73, 48, 0], [4, 8, 97, 17, 9, 7, 0], [4, 8, 54, 73, 73, 54, 0], [4, 8, 6, 73, 73, 62, 0], [2, 8, 80, 0, 0, 0, 0], [2, 8, 128, 80, 0, 0, 0], [3, 8, 16, 40, 68, 0, 0], [3, 8, 20, 20, 20, 0, 0], [3, 8, 68, 40, 16, 0, 0], [4, 8, 2, 89, 9, 6, 0], [5, 8, 62, 73, 85, 93, 14], [4, 8, 126, 17, 17, 126, 0], [4, 8, 127, 73, 73, 54, 0], [4, 8, 62, 65, 65, 34, 0], [4, 8, 127, 65, 65, 62, 0], [4, 8, 127, 73, 73, 65, 0], [4, 8, 127, 9, 9, 1, 0], [4, 8, 62, 65, 73, 122, 0], [4, 8, 127, 8, 8, 127, 0], [3, 8, 65, 127, 65, 0, 0], [4, 8, 48, 64, 65, 63, 0], [4, 8, 127, 8, 20, 99, 0], [4, 8, 127, 64, 64, 64, 0], [5, 8, 127, 2, 12, 2, 127], [5, 8, 127, 4, 8, 16, 127], [4, 8, 62, 65, 65, 62, 0], [4, 8, 127, 9, 9, 6, 0], [4, 8, 62, 65, 65, 190, 0], [4, 8, 127, 9, 9, 118, 0], [4, 8, 70, 73, 73, 50, 0], [5, 8, 1, 1, 127, 1, 1], [4, 8, 63, 64, 64, 63, 0], [5, 8, 15, 48, 64, 48, 15], [5, 8, 63, 64, 56, 64, 63], [5, 8, 99, 20, 8, 20, 99], [5, 8, 7, 8, 112, 8, 7], [4, 8, 97, 81, 73, 71, 0], [2, 8, 127, 65, 0, 0, 0], [4, 8, 1, 6, 24, 96, 0], [2, 8, 65, 127, 0, 0, 0], [3, 8, 2, 1, 2, 0, 0], [4, 8, 64, 64, 64, 64, 0], [2, 8, 1, 2, 0, 0, 0], [4, 8, 32, 84, 84, 120, 0], [4, 8, 127, 68, 68, 56, 0], [4, 8, 56, 68, 68, 40, 0], [4, 8, 56, 68, 68, 127, 0], [4, 8, 56, 84, 84, 24, 0], [3, 8, 4, 126, 5, 0, 0], [4, 8, 152, 164, 164, 120, 0], [4, 8, 127, 4, 4, 120, 0], [3, 8, 68, 125, 64, 0, 0], [4, 8, 64, 128, 132, 125, 0], [4, 8, 127, 16, 40, 68, 0], [3, 8, 65, 127, 64, 0, 0], [5, 8, 124, 4, 124, 4, 120], [4, 8, 124, 4, 4, 120, 0], [4, 8, 56, 68, 68, 56, 0], [4, 8, 252, 36, 36, 24, 0], [4, 8, 24, 36, 36, 252, 0], [4, 8, 124, 8, 4, 4, 0], [4, 8, 72, 84, 84, 36, 0], [3, 8, 4, 63, 68, 0, 0], [4, 8, 60, 64, 64, 124, 0], [5, 8, 28, 32, 64, 32, 28], [5, 8, 60, 64, 60, 64, 60], [5, 8, 68, 40, 16, 40, 68], [4, 8, 156, 160, 160, 124, 0], [3, 8, 100, 84, 76, 0, 0], [3, 8, 8, 54, 65, 0, 0], [1, 8, 127, 0, 0, 0, 0], [3, 8, 65, 54, 8, 0, 0], [4, 8, 8, 4, 8, 4, 0]]
}
function digit_test()  {
    for (let q = 0; q <= font.length - 1; q++) {
        copy_font_to_buffer(q)
ledmatrix.write_buffer(0, 0, buffer, 5)
// copy_font_to_buffer(p) ledmatrix.write_buffer(8, 0,
        // buffer)
        basic.showNumber(q % 10, 100)
    }
}
function digit_random()  {
    digits = [0, 0]
    digits[0] = Math.random(font.length - 1 + 1)
    digits[1] = Math.random(font.length - 1 + 1)
    copy_font_to_buffer(digits[0])
ledmatrix.write_buffer(0, 0, buffer, 8)
// copy_font_to_buffer(digits[1])
    // ledmatrix.write_buffer(8, 0, buffer)
    basic.showNumber(p % 10)
    p += 1
}
function init()  {
    basic.showNumber(0)
    serial.writeLine("on start()")
    digits = []
    font_init()
    ledmatrix.init()
ledmatrix.setIntensity(1);
ledmatrix.write_buffer(0, 0, buffer);
basic.showNumber(1)
    serial.writeLine("on start()end")
}
let digits: number[] = []
let buffer: number[] = []
let font: number[][] = []
// let ledmatrix = new
// LedMatrix_MAX7219(DigitalPin.P15, DigitalPin.P13,
// DigitalPin.P16, 2);
let ledmatrix = new LedMatrix_MAX7219(DigitalPin.P15, DigitalPin.P13, DigitalPin.P16);
// MAX7219_REG enum the opcodes for the MAX7221 and
// MAX7219
enum MAX7219_REG {
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
;
class LedMatrix_MAX7219 {
    pin_dta: number;
    pin_clk: number;
    pin_cs: number;
    maxDevices: number;
    dir_mode: number;
    buffer: number[];
    font_buffer_unit_size: number;

    constructor(
        _pin_dta: DigitalPin,
        _pin_clk: DigitalPin,
        _pin_cs: DigitalPin,
        _device_num: number = 1) {
        // Const enum DigitalPin
        this.pin_dta = _pin_dta;
        this.pin_clk = _pin_clk;
        this.pin_cs = _pin_cs;

        if (_device_num <= 0 || _device_num > 8)
            _device_num = 8;
        this.maxDevices = _device_num;

        this.dir_mode = 0;
        //this.buffer.length = 80;
        this.font_buffer_unit_size = 7;
    }

    init() {
        pins.digitalWritePin(this.pin_dta, 1)
        pins.digitalWritePin(this.pin_clk, 1)
        pins.digitalWritePin(this.pin_cs, 1)

        this.set_command(MAX7219_REG.SCAN_LIMIT, 0x07);
        this.set_command(MAX7219_REG.DECODE_MODE, 0x00);  // using an led matrix (not digits)
        this.set_command(MAX7219_REG.SHUTDOWN, 0x01);     // not in shutdown mode
        this.set_command(MAX7219_REG.DISPLAY_TEST, 0x00); // no display test

        // empty registers, turn all LEDs off
        this.clearDisplay();

        this.setIntensity(0x0f);    // the first 0x0f is the value you can set
    }

    // width -> specify the data width 
    //          *default read by buffer first byte
    write_buffer(x: number, y: number, _buffer: number[], width: number = 0) {
        let c = 0
        let r = 0
        let w = buffer[0];
        let h = buffer[1];

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
                    this.set_column(c, _buffer[i + 2])
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
        for (let j = 0; j < 8; j++)
            this.set_column(j, 0)
    }

    getDeviceCount(): number {
        return this.maxDevices;
    }

    set_column(col: number, data: number) {
        let n = col / 8
        let d = col % 8
        pins.digitalWritePin(this.pin_cs, 0)
        for (let k = 0; k < this.maxDevices; k++) {
            if (k == n) {
                this.set_command(d + 1, data)
            } else {
                this.set_command(0, 0)
            }
        }
        pins.digitalWritePin(this.pin_cs, 0)
        pins.digitalWritePin(this.pin_cs, 1)
    }

    set_command(command: number, value: number) {
        // set CS_PIN = LOW
        pins.digitalWritePin(this.pin_cs, 0)
        for (let l = 0; l < this.maxDevices; l++) {
            this.write_byte(command)
            this.write_byte(value)
        }
        // set CS_PIN = HIGH
        pins.digitalWritePin(this.pin_cs, 0)
        pins.digitalWritePin(this.pin_cs, 1)
    }

    write_byte(value: number) {
        for (let m = 0; m < 8; m++) {
            pins.digitalWritePin(this.pin_clk, 0) // CLOCK_PIN = LOW
            pins.digitalWritePin(this.pin_dta, ((value << m) & 0x80) >> 7) // DATA_PIN
            pins.digitalWritePin(this.pin_clk, 1) // CLOCK_PIN = HIGH
        }
        pins.digitalWritePin(this.pin_dta, 1)
    }

    // cmd: 0x09, value : 0x0~
    setDecodeMode(value: number) {
        this.set_command(MAX7219_REG.DECODE_MODE, value)
    }
    // cmd: 0x0A, value : 0x0~0xF
    setIntensity(value: number) {
        this.set_command(MAX7219_REG.INTENSITY, value);
    }
    // cmd: 0x0B, value : 0x0~0x7
    setScanLimit(value: number) {
        this.set_command(MAX7219_REG.SCAN_LIMIT, value)
    }
    // cmd: 0x0C, value : 0x0~0x01
    setShutdownMode(value: number) {
        this.set_command(MAX7219_REG.SHUTDOWN, value)
    }
    // cmd: 0x0F, value : 0x0~0x01
    set_display_test(value: number) {
        this.set_command(MAX7219_REG.DISPLAY_TEST, value)
    }
    // max7219_set_dir() value -> set and 0x01 : vertical
    // invert -> set and 0x02 : horizontal invert
    setDirMode(value: number) {
        this.dir_mode = value
    }


}
function copy_font_to_buffer(param_font_idx: number) {
    // check
    if (param_font_idx < 0)
        return
    for (let o = 0; o <= 7 - 1; o++) {
        buffer[o] = font[param_font_idx][o]
    }
}
init()
basic.forever(() => {
    digit_test()
})
