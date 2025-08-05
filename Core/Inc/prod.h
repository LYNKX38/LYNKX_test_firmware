/*
 * prod.h
 *
 *  Created on: Jun 1, 2023
 *      Author: robin
 */

#ifndef INC_PROD_H_
#define INC_PROD_H_

/* Defines */
/*
#define LED_GREEN1_Pin GPIO_PIN_0
#define LED_GREEN1_GPIO_Port GPIOB
#define LED_GREEN2_Pin GPIO_PIN_5
#define LED_GREEN2_GPIO_Port GPIOC
#define LED_RED1_Pin GPIO_PIN_9
#define LED_RED1_GPIO_Port GPIOB
#define LED_RED2_Pin GPIO_PIN_0
#define LED_RED2_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define BUZZER_EN_Pin GPIO_PIN_6
#define BUZZER_EN_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOB*/

/* Functions Prototypes */
void Test_LED_GREEN1(void);
void Test_LED_GREEN2(void);
void Test_LED_RED1(void);
void Test_LED_RED2(void);
void Test_LED_Flash(void);
void Test_BC_State(void);
void Test_Pressure(void);
void Test_ACC(void);
void Test_Sound(void);
void Test_BLE(void);
void Test_LoRa(void);
void Test_GNSS(void);
void Test_Flash(void);
void Test_Emergency(void);


uint8_t Read_ACC_WHO_AM_I(void);
void Play_Sound(uint16_t freq, uint16_t duration);


#endif /* INC_PROD_H_ */
