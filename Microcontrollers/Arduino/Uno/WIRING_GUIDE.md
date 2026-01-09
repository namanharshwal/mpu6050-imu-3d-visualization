# MPU6050 Arduino Uno Wiring Guide

## Quick Reference

### Pin Connection Summary

| Arduino Pin | MPU6050 Pin | Description | Cable Color |
|-------------|------------|-------------|-------------|
| 5V | VCC | Power Supply | Red |
| GND | GND | Ground | Black |
| A4 (SDA) | SDA | I2C Data Line | Green |
| A5 (SCL) | SCL | I2C Clock Line | Yellow |
| D2 | INT | Interrupt (Optional) | Blue |
| GND | AD0 | Address Select | Black |

---

## Detailed Wiring Instructions

### Step 1: Power Connections

**Arduino to MPU6050 Power Supply**

```
Arduino 5V Pin  ----[Red Wire]---- MPU6050 VCC Pin
```

- Connect the **5V pin** from Arduino directly to the **VCC (3.3V/5V)** pin on MPU6050
- Use a **red wire** for easy identification
- **Important**: MPU6050 can typically handle 5V input (check your module)
  - If using 3.3V module, use an external 3.3V regulator

**Arduino GND Connection**

```
Arduino GND Pin  ----[Black Wire]---- MPU6050 GND Pin
```

- Connect **any GND pin** from Arduino to **GND** on MPU6050
- Use a **black wire** for ground
- **Critical**: Both devices must share the same ground reference

---

### Step 2: I2C Communication Wiring

**I2C Protocol Setup**

Arduino Uno uses **I2C (Two-Wire Interface)** for communication with MPU6050.

```
Arduino A4 (SDA)  ----[Green Wire + 4.7k Resistor]---- MPU6050 SDA
                       |
                      +5V (via resistor)

Arduino A5 (SCL)  ----[Yellow Wire + 4.7k Resistor]---- MPU6050 SCL
                       |
                      +5V (via resistor)
```

#### Pull-up Resistor Installation

**Why Pull-up Resistors?**
- I2C is an open-drain protocol requiring pull-up resistors
- Arduino Uno has internal 20k resistors, but they may be insufficient
- External 4.7k resistors improve signal integrity

**Installation Method:**

1. **Resistor on SDA Line:**
   - One end of 4.7k resistor => Arduino A4 (SDA) and MPU6050 SDA junction
   - Other end => Arduino 5V

2. **Resistor on SCL Line:**
   - One end of 4.7k resistor => Arduino A5 (SCL) and MPU6050 SCL junction  
   - Other end => Arduino 5V

---

### Step 3: Interrupt Pin (Optional)

```
Arduino D2 (INT)  ----[Blue Wire]---- MPU6050 INT Pin
```

- Connect MPU6050 **INT** (interrupt) pin to Arduino **D2**
- This enables data-ready interrupts for synchronous reading
- Optional: Can use polling instead if not connected

---

### Step 4: Address Selection

```
MPU6050 AD0 Pin  ----[Black Wire]---- Arduino GND
```

- Connect **AD0** pin to **GND**
- This sets I2C address to **0x68** (default)
- If pulled to 5V instead, address becomes **0x69**

---

## Component Checklist

Before wiring, gather these components:

- [ ] Arduino Uno Rev3
- [ ] MPU6050 GY-521 Module
- [ ] 2x 4.7k Resistors (1/4W, 5% tolerance)
- [ ] Jumper wires (male-to-male):
  - [ ] Red wire (Power)
  - [ ] 2x Black wires (Ground)
  - [ ] Green wire (SDA)
  - [ ] Yellow wire (SCL)
  - [ ] Blue wire (INT - optional)
- [ ] Breadboard (optional)
- [ ] USB Cable (Type-B for Arduino)

---

## Wiring Best Practices

### Cable Management

1. **Use Color-Coded Wires:**
   - Red = Power (5V)
   - Black = Ground (GND)
   - Green = SDA (Data)
   - Yellow = SCL (Clock)
   - Blue = Interrupt

2. **Keep I2C Lines Short:**
   - Minimize SDA/SCL cable length
   - Long cables increase capacitance and can cause signal issues
   - Recommended: < 1 meter for prototyping

3. **Avoid Noise Sources:**
   - Keep I2C wires away from high-current lines
   - Don't bundle I2C with power wires
   - Use shielded cables for long runs

---

## Decoupling Capacitors (Noise Reduction)

For robust operation, add capacitors close to MPU6050:

- **C1 (0.1uF)**: Ceramic capacitor for high-frequency noise filtering
- **C2 (10uF)**: Electrolytic capacitor for bulk charge storage

Place C1 as close as possible to VCC/GND pins (~5mm)

---

## Troubleshooting Wiring Issues

### Issue: "Could not find MPU6050"

**Possible Causes:**
1. **Loose connections**
   - Verify all wires are fully inserted
   - Check breadboard contact pins are clean

2. **Wrong pins used**
   - Confirm using A4 (SDA) and A5 (SCL)
   - Check D2 for interrupt (if used)

3. **Missing pull-up resistors**
   - Verify 4.7k resistors installed
   - Test with multimeter: Should show ~2.5k from SDA/SCL to GND

4. **Wrong I2C address**
   - Check if AD0 is connected to GND (0x68) or 5V (0x69)

### Issue: Intermittent Sensor Readings

**Possible Causes:**
1. **Loose breadboard connections**
   - Press down on all jumper wires
   - Replace worn breadboard

2. **Weak pull-ups**
   - Verify resistor values (should be 4.7k +/- 10%)
   - Check resistor continuity with multimeter

3. **EMI/Noise**
   - Add 0.1uF capacitor near VCC
   - Use shielded I2C cables
   - Keep away from USB and power cables

---

## Testing Your Wiring

### Quick Test Procedure

1. **Visual Inspection (No Power)**
   - Check all connections match wiring diagram
   - Verify no wires crossing each other
   - Confirm pull-up resistors installed

2. **Continuity Test (Multimeter)**
   - 5V to VCC: Should be continuous
   - GND to GND (all): Should be continuous
   - A4 to SDA: Should be continuous
   - A5 to SCL: Should be continuous
   - D2 to INT: Should be continuous

3. **Voltage Test (Powered On)**
   - VCC: 4.75-5.25V
   - SDA: Should be 5V (pulled high)
   - SCL: Should be 5V (pulled high)
   - AD0 voltage: Should be ~0V (grounded)

---

## References

- **MPU6050 Datasheet**: https://www.onsemi.com/pub/Collateral/MPU-6000-Datasheet1.pdf
- **Arduino Uno Pin Reference**: https://www.arduino.cc/en/Reference/Wire
- **I2C Specification**: https://www.nxp.com/docs/en/user-guide/UM10204.pdf

**Last Updated**: January 2026
**Version**: 1.0
**Author**: Naman Harshwal
