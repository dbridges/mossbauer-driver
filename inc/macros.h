#ifndef MACROS_H
#define MACROS_H

#define TRUE                        1
#define FALSE                       0

#define HIGH                        1
#define LOW                         0


#define GREEN                       GPIO_Pin_12
#define ORANGE                      GPIO_Pin_13
#define RED                         GPIO_Pin_14
#define BLUE                        GPIO_Pin_15
#define ALL_LEDS                    (GREEN | ORANGE | RED | BLUE)
#define PUSHBUTTON                  GPIO_Pin_0

#define LED_PORT                    (GPIOD)
#define PUSHBUTTON_PORT             (GPIOA)

#define STATE_LOOKING_FOR_DARK      0
#define STATE_LOOKING_FOR_LIGHT     1

#endif
