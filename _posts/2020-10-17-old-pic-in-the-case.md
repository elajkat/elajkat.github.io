---
layout: post
title: "Old pic (PIC16f690 and PICKit2) in the case"
date: 2020-09-30
categories: pic tech tutorial
---


Recently I had to move some of my old stuff from a desk, and in one of the cases I found a pickit2 and a small demo board with a pic16F690 on it.
I decided to do some similar simple demo like it is common for microbits: blinking LEDs, use some push buttons, and the potentio meter. 
I fetched my old knowledge of pics and generally microcontrollers, and of course C language.

## Hardware Ingredients

* You can still download the user's guide for the demo board: [Low Pin Count Demo Board User’s Guide](http://ww1.microchip.com/downloads/en/devicedoc/low%20pin%20count%20user%20guide%2051556a.pdf)

{{site.data.alerts.warning}}
<pre>
PICKit2 is depreceated (see: <a href="https://www.microchip.com/DevelopmentTools/ProductDetails/PartNO/PG164120">PICkit 2 Development Programmer/Debugger</a> )
</pre>
{{site.data.alerts.end}}

* The demo board for some reason connects 
  * the push button to RA3 which is used for programming
  * the potentiometer to RA0 which is again used for programming.

So I decided to change this a little, and connected

* the push button to RA5
* the potentiometer to RA2

<img src="/assets/old_pic_in_the_case/IMG_20201010_140843.jpg" width="200">
<img src="/assets/old_pic_in_the_case/IMG_20201010_135343.jpg" width="200">

I have only an old soldering-iron and some old wires at home so this is why the result is far from nice.

## Software ingredients

My goal was to do something similar like the microbit basic projects, blinking LEDs, do something with the push button or with the potentiometer.
I quickly come to the conclusion that since I last used not just the PICKit2 itself became deprecated, but the development environment for it.
At that time I used a Windows as OS, now I use linux (Ubuntu 20.04 actually), so I had to find out

* what kind of IDE shall I use to have device specific .h files (in the past I used MPLAB IDE, which was a windows only tool)
* what kind of builder shall I used (it was MLAB as well with integrated builder)
* how to download my binary to the PIC's program memory (this was MPLAB again in the past)

I started to look for some opensource solution for it, but soon I found a working constellation.

* MPLABX is the new netbeans based IDE proposed by Microchip, and that works on linux.
* MPLABX has all the things to link and build all device specific things into a binary, and give help (code complition, etc...) for coding.
* MPLABX can connect to various tools, like programmers, or debuggers
* But not for eons old PICKit2, but luckily there's an old project wich give CLI for PICKit2, pk2cmd.

### pk2cmd

{{site.data.alerts.warning}}
<pre>
pk2cmd is not officially supported by Microchip, 
so no guarantee that it works in all circumtances.
</pre>
{{site.data.alerts.end}}

Here is a quite informative page which describes how to build it from source on linux:
* http://curuxa.org/en/Pk2cmd_manual_installation, it worked me on Ubuntu 20.04.

The tool itself suffers some terrible old windows CLI influence, but that's just my personal feeling perhaps.
For usage tips check the README file, or check this page: 
[Programming PICs using the PICKit2 and pk2cmd](http://www.waveguide.se/?article=programming-pics-using-the-pickit2-and-pk2cmd)

## Let's do it

### Start a project in MPLABX for PIC16f690

To start the project some clicking is needed, for help suprisingly there's a lot of help on Microchip's pages.
Basically Click on **Start New Project**, select **Microchip Embedded**, and **Standalone Project**,
click next, and there you can select which family (Mid Range 8bit MCU for filtering can help) and 
voila PIC16f690 is in the list. i
The same with screenshots: [Lab 1 - Creating an MPLAB X Project](https://microchipdeveloper.com/tls0101:lab1)

### General configuration

Now that we have a fancy project for our beloved MCU, let's setup the configuration word for it.
Here comes the hard part, with a lot of reading.
* [PIC16F631/677/685/687/689/690 Data Sheet](http://ww1.microchip.com/downloads/en/devicedoc/41262c.pdf)

Check Configuration Bits section (14.1 in my downloaded pdf)

{% highlight c %}
#include <xc.h>
#include <pic.h>

#pragma config FOSC = INTRCIO   // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)  
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)  
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT disabled)  
#pragma config MCLRE = OFF       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)  
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)  
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)  
#pragma config BOREN = OFF       // Brown-out Reset Enable (Brown-out Reset enabled)  
#pragma config IESO = OFF        // Internal/External Switchover (Internal/External Switchover mode is enabled)  
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)  
{% endhighlight %}

The important part is to 
* select the clock source (section 3.0), for now I selected internal clock source.
* and I switched of all other feature now as it is a small hoby project only.

For clock frequency selection OSCCON register must be set with the necessary values.

### LED blinking

Blinking LEDs from software is easy, you check which pin is connected to the LED, add a loop and some delay.
(Note: This is true for both AD conversion, and for puch button check)
Even there are available functions you can use, something like this:

{% highlight c %}
#include <xc.h>
#include <pic.h>
#define _XTAL_FREQ 8000000
void main(void) {
    __delay_ms(1);
    // Blink the LED;
}
{% endhighlight %}

**BUT** we have a smart MCU, with builtin hardware units, so use these, for blinking use one of the
timer modules (PIC16f690 has 3 of these timers), and use **interrupts**.

#### Interrupts

An interrupt (interrupt routin) is a response to some **event**. The event can be **external** (like change of a pin's state,
or changing the value on an ADC's or converter's input) or **internal** (like a timer's counter reaches it's predefined value).

Related wikipedia article: [Interrupt](https://en.wikipedia.org/wiki/Interrupt)

For the available interrupts for the given PIC, check the datasheet always.

#### LED blinking with interrupt

For this small project I selected Timer0 of PIC16f690, which is perfect for this purpose.

Timer0 is an 8-bit timer/counter, with 8-bit prescaler, selectable internal or external
clock source, with interrupt on overflow.

To setup Timer0 check the datasheet (as usual), for every section that describes some 
ardware element there is a summary of registers that is associated with it:

From TABLE 5-1: SUMMARY OF REGISTERS ASSOCIATED WITH TIMER0

| Name | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|------|------|------|------|------|------|------|------|------|
| INTCON | GIE |     | T0IE |      |      | T0IF |      |      |
| OPTION_REG | |     | T0CS | T0SE | PSA  | PS2  | PS1  | PS0  |   
| TRISA |    |       |      |      |      | TRISA2 |    |      |

TMR0 (Timer0 Module Register) is the one to set the value for the timer.

Code snippet to setup Timer0:

{% highlight c %}
void main(void) {
    OPTION_REG = 0b00000111;  // Timer0 Internal instruction cycle clock (F OSC /4)
                              // 256 as prescalar 
                              // Also Enables PULL UPs
    
    // Setup timer0 with interrupt
    TMR0 = 1;  // Load the time value for 0.0019968s; delayValue can be between 0-256 only
    T0IE = 1;  // Enable timer interrupt bit in INTCON register
    T0IF = 0;  // Clear Overflow bit (must be cleared in software)
    
    // Setup Portc as output
    TRISC = 0x00;
    PORTC = 0b00000000; //Initialize all pins to 0
    
    GIE = 1;              //Enable Global Interrupt

   // Do something else....
}
{% endhighlight %}

What is next to catch that interrupt. In MplabX you have to define an itterrupt routine for that.

The interrupt code is just code like all other code, but there is a special address in the program memory
that holds the address to go if an interrult happens, see the memory organization in the datasheet:

![isdfs](/assets/old_pic_in_the_case/Program_mem_map.png )

The **Reset Vector** is the special address in program memory to where the code execution jumps if an iterrupt
happens. In assembly we should add a jump command there to jump to our interrupt handling routine, in C, it is
the compiler's responsibility to put everything in place in the hex file that is downloaded to the device's
program memory.

We can live without assembly, and memory bank definitions and similar terrible things (come on I work with python),
so the **c** language solution for that with MplabX (and XC compiler):

{% highlight c %}
void __interrupt() tc_int(void)
{
    GIE = 0;  // Disable interrupts to be sure that no
              // new interrupt interrupts use here

    if(T0IE && T0IF) // Timer flag has been triggered due
                     // to timer overflow
    {
        TMR0 = 1;    // Load the timer Value
        count++;
        T0IF = 0;    // Clear timer interrupt flag
    }

    GIE = 1;  // Don't forget to enable again all interrupts!
}  
{% endhighlight %}

In this code snippet I do not run to PortC pins to change their value,
just increase a counter variable **count**, this can give use the
possiblilty to increase the delay as Timer0 is only an 8 bit timer.

To count the necessary values based on clock frequency and prescaler values 
check this page for example:
http://www.piclist.com/techref/microchip/timer.htm

The general formula to count the TMR0 value:

``TMR0 = 256 - ((Delay * Fosc)/(Prescaler * 4))``

{{site.data.alerts.note}}
<pre>
Delay in sec and Fosc in Hz
</pre>
{{site.data.alerts.end}}

This value can be stretched by adding more counters that can be increased and checked in the main loop.

### Using a push button

Checking the state of an input pin can be done by checking the input value in the main loop, that is 
again a software solution, but the main loop should have some more interesting thing to do, and we have 
hardware for doing pin value check.

So let's use an interrupt for this as well.

I connected the push button on the demo board to RA5 (instead of the original RA3 which is used by PICkit2 for 
programming), as every PortA pins has **interrupt-on-change** functionality we can use that to detect if the 
push button was pressed or not.

First check the *datasheet* as usual, the interesting part now is I/O Ports and PortA and TrisA registers and 
4.2.3 INTERRUPT-ON-CHANGE.

Code snippet to setup RA5 as interrupt source:

{% highlight c %}
void main(void) {
    // Setup Porta5 as input with interrupt-on-change
    TRISA5 = 1;      // RA5 input
    IOCA5 = 1;       // RA5 interrupt enabled

    PEIE = 1;        // Enable the Peripheral Interrupt
    GIE = 1;         // Enable Global Interrupt

    // Do something else...
}
{% endhighlight %}

Next thing is to add the logic to the interrupt logic to check RA5 interrupt as well:

{% highlight c %}
unsigned char toggle_rc3 = 0;

void __interrupt() tc_int(void)
{
    GIE = 0;  // Disable interrupts to be sure that no
              // new interrupt interrupts use here

    if (IOCA5 && RABIF)
    {
        if (RA5 == 0)
            toggle_rc3 ^= 1;
        if (toggle_rc3)
                write_to_rc3 = 0b00001000;
        else
            write_to_rc3 = 0b00000000;
        PORTC = write_to_rc3;
        RABIF = 0;    // Clear the interrupt flag bit
    }
    
    GIE = 1;  // Don't forget to enable again all interrupts!
}
{% endhighlight %}

My logic here contains some extra steps to handle that I use the LED connected to 
RC3 to blink by the push button and the other pins of PORTC (RC0..RC2) for the blinking.

### Uning a potentiometer

After we have all the knowledge for interrupts and digital I/O, analog input will be much simpler.

PIC MCUs and PIC16f690 has builtin A/D converters, and with some configuration and care 
(read the datasheet!) we can make use of it to grab the input voltage on a pin and do something
accordingly.

As I wrote, I connected the potentiometer to RA2 pin (instead of the original RA0 which is used by PICkit2)

The following code snippet configures the registers for making RA2 an analog input:

{% highlight c %}
void main(void) {
    // Setup ADC with interrupt
    TRISA2 = 1;
    ANS2 = 1;           // RA2 analog input
    ADCON0 = 0b10001001;// Right justified, AN2 channel, AD enabled
    ADCON1 = 0b00100000;// A/D Conversion Clock FOSC/32
    ADIF = 0;           // Clear AD conversion flag bit, PIR1 register
    ADIE = 1;           // A/D Converter (ADC) Interrupt Enable PIE1 register

    GIE = 1;            // Enable Global Interrupt
    GO_nDONE = 1;       // A/D conversion cycle in progress, setting this
                        // starts and A/D conversion cycle.      
}
{% endhighlight %}

The logic in the interrupt handler:

{% highlight c %}
unsigned int ad_value = 0;

void __interrupt() tc_int(void)
{
    if (ADIE && ADIF && !GO_nDONE)
    {
        // Note: this can work only as the result is right justified,
        // as ADFM in ADCON0 is 1
        ad_value = ((ADRESH << 8) + ADRESL); 
        ADIF = 0;
        ADIE = 1;
        GO_nDONE = 1;
    }

    GIE = 1;    // Don't forget to enable again all interrupts!
}

{% endhighlight %}

## The code

You can find the code on github:

https://github.com/elajkat/pic_led_demo

## Downloading the code to the device

It will again some stone age process that comes here, compared to microchip for example.
I have to say to though that you can easily program PICs with bootloaders.

if you use MPLABX as IDE, it is not that simple to find the hex file after compilation,
I work on Linux, and my project is located at my home under `MPLABXProjects`, so after
successful compilation the hex file's path:

`~/MPLABXProjects/dist/default/production/MPLABXProjects.production.hex`

So to program the hex file:

`pk2cmd -P PIC16F690 -X -M -F MPLABXProjects.production.hex`

You can see that the ``Busy`` and ``Target`` LEDs on PICKit2 are blinking during the process.

For this demo I used only the PICKit2 to power the demo board, and pkcmd2 can help in that too:

``pk2cmd -P PIC16F690 -T``

To power off the board use:

``pk2cmd -P PIC16F690`` 

For some reason this doesn't work for me.

## Summary

Working with microcontrollers is more electrical engineering, and much more 
background knowledge is needed to understand what to touch and twirl.

My goal was to show that it is doable to create similar things with a PIC, in
a low level, sometimes more assembly than C language.

