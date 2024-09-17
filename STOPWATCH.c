#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

#define SET_BIT(REG , BIT) (REG |= (1<<BIT))
#define CLR_BIT(REG , BIT) (REG &= ~(1<<BIT))


unsigned char tick = 0;
unsigned char display_digit = 0;


//variables to track button state, initially released.
unsigned char sec_inc_button = 1;
unsigned char sec_dec_button = 1;

unsigned char min_inc_button = 1;
unsigned char min_dec_button = 1;

unsigned char hours_inc_button = 1;
unsigned char hours_dec_button = 1;


unsigned char sec_ones = 0;
unsigned char sec_tens = 0;
unsigned char min_ones = 0;
unsigned char min_tens = 0;
unsigned char hours_ones = 0;
unsigned char hours_tens = 0;


void io_init(void)
{
	DDRA |= 0x3F;
	DDRB = 0x00;    //PORTB i/p
	DDRC |= 0x0F;
	DDRD |= 0x31;    //PD0, PD4, PD5 o/p
	DDRD &= ~(0x0C); //PD2,PD3 i/p


	PORTD &= ~(1<<PD3);



	PORTA &= ~(0xCF);        //Disable all 7-segments
	PORTB |= 0x7F;          //activate pull-up resistor for PB0, PB1, PB2, PB3, PB4, PB5, PB6
	PORTC &= ~(0x0F);      //clear first 4-bits to set decoder pins HIGH

	SET_BIT(PORTD , PD2); //Activate internal pull-up resistor

}


void timer1_init(void)
{
	TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS11);  //CTC mode, 64 prescaler

	OCR1A = 2499; //set OCR1A for 10ms interrupts
	             // OCR1A = (Fcpu / (prescaler * Frequency)) - 1
	            // OCR1A = (16000000 / (64 * 100)) - 1 = 2499

	TIMSK |= (1<<OCIE1A); //Enable interrupt for Timer1

	SREG |= (1<<7);  //Global interrupt enable
}

ISR (TIMER1_COMPA_vect)
{
	tick++;

	if(tick == 100) //Display on 7-segment every 1 second (100 * 10ms = 1 sec)
	{
		display_digit = 1;

		tick = 0;
	}
}

void segment_init (void)
{
	if (! (PINB & (1<<PB7)) )
	{
		_delay_ms(30);

		if (! (PINB & (1<<PB7)) )  //To overcome de-bounce
		{
			SET_BIT( PORTD, PD4 );   //Activate Red LED during count up

			sec_ones++;

			if (sec_ones == 10)
			{
				sec_ones = 0;
				++sec_tens;

				if (sec_tens == 6)
				{
					sec_tens = 0;
					++min_ones;

					if(min_ones == 10)
					{
						min_ones = 0;
						++min_tens;

                        // Check if hours are 24 (i.e., 23:59:59 has passed)
                        if (hours_ones == 4 && hours_tens == 2)
                        {
                            // Reset to 00:00:00 after reaching 24 hours
                            hours_tens = 0;
                            hours_ones = 0;
                            min_tens = 0;
                            min_ones = 0;
                            sec_tens = 0;
                            sec_ones = 0;
                        }
                        else if (hours_ones == 10)
                        {
                            hours_ones = 0;
                            ++hours_tens;
                        }

					}
				}
			}
		}
	}

	else
	{
		CLR_BIT( PORTD, PD4 );  //De-activate RED LED during count down
		SET_BIT( PORTD, PD5 );   //Activate Yellow LED during count up

		if(! (sec_ones || sec_tens || min_ones || min_tens || hours_ones || hours_tens) )
		{
			SET_BIT (PORTD, PD0); //Activate Buzzer
		}

		else
		{

			if(sec_ones != 0)
			{
				--sec_ones;
			}
			else
			{
				sec_ones = 9;
				if(sec_tens != 0)
				{
					--sec_tens;
				}
				else
				{
					sec_tens = 5;
					if(min_ones != 0)
					{
						--min_ones;
					}
					else
					{
						min_ones = 9;
						if(min_tens != 0)
						{
							--min_tens;
						}
						else
						{
							min_tens = 5;
							if(hours_ones != 0)
							{
								--hours_ones;
							}
							else
							{
								hours_ones = 9;
								if(hours_tens != 0)
								{
									--hours_tens;
								}
							}
						}
					}
				}
			}
		}
	}

	display_digit = 0; //stop displaying till the next second
}

void INT0_init(void)  //Interrupt to reset the timer
{
	MCUCR |= (1<<ISC01);  //Trigger Interrupt 0 with falling edge
	GICR  |= (1<<INT0);
}

ISR(INT0_vect)
{

	//Reset all 7-segments
	sec_ones = 0;
	sec_tens = 0;
	min_ones = 0;
	min_tens = 0;
	hours_ones = 0;
	hours_tens = 0;


	CLR_BIT (PORTD, PD0); //Stop Alarm

}

void INT1_init(void) //Interrupt to pause the timer
{
	MCUCR |= (1<<ISC10) | (1<<ISC11);  //Trigger Interrupt 1 with rising edge
	GICR  |= (1<<INT1);
}

ISR(INT1_vect)
{

	CLR_BIT (TCCR1B, CS10); //Clear all clock-bits to stop the timer
	CLR_BIT (TCCR1B, CS11);
	CLR_BIT (TCCR1B, CS12);

}

void INT2_init()  //Interrupt to resume the timer
{
	CLR_BIT(MCUCSR, ISC2);  //Trigger Interrupt 2 with falling edge
	GICR  |= (1<<INT2);
}

ISR(INT2_vect)
{
	//set clock-bits to resume the timer
	SET_BIT (TCCR1B, CS10);
	SET_BIT (TCCR1B, CS11);
}


int main (void){

	io_init();
	timer1_init();

	INT0_init();
	INT1_init();
	INT2_init();


	while(1)
	{
		PORTA = (PORTA & 0xC0) | (1<<PA5);            //Activate first 7-segment
		PORTC = (PORTC & 0xF0) | (sec_ones & 0x0F);  //Display seconds ones
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | (1<<PA4);            //Activate second 7-segment
		PORTC = (PORTC & 0xF0) | (sec_tens & 0x0F);  //Display seconds tens
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | (1<<PA3);            //Activate third 7-segment
		PORTC = (PORTC & 0xF0) | (min_ones & 0x0F);  //Display Minutes ones
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | (1<<PA2);            //Activate fourth 7-segment
		PORTC = (PORTC & 0xF0) | (min_tens & 0x0F);  //Display Minutes tens
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | (1<<PA1);             //Activate fifth 7-segment
		PORTC = (PORTC & 0xF0) | (hours_ones & 0x0F); //Display hours ones
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | (1<<PA0);             //Activate sixth 7-segment
		PORTC = (PORTC & 0xF0) | (hours_tens & 0x0F); //Display hours tens
		_delay_ms(2);

/*************************************************/

		if (display_digit == 1)
		{
			segment_init();
		}

		if(! (PINB & (1<<PB6)))     //Overcome de-bounce
		{
			_delay_ms(30);

			if(! (PINB & (1<<PB6)))
			{
				if(sec_inc_button)  //Only increment if the button was previously released
				{
					sec_ones++;

					if(sec_ones >= 10)
					{
						sec_ones = 0;
						sec_tens++;
					}

					if (sec_tens >= 6)
					{
						sec_tens = 0;
					}

					sec_inc_button = 0;   //Button is now pressed, stop incrementing
				}
			}
		}
		else
		{
			sec_inc_button = 1; //Reset button to original released state, to increment again next time.
		}


		if(! (PINB & (1<<PB5)))
		{
			if(! (PINB & (1<<PB5))) //overcome de-bounce
			{
				if(sec_dec_button)  //Only decrement if the button was previously released
				{

					if(sec_ones != 0)
					{
						sec_ones--;
					}
					else if(sec_tens != 0)
					{
						sec_ones = 9;
						sec_tens--;
					}
					else
					{
						//to prevent timer from decrementing after 00
					}

					sec_dec_button = 0;   //Button is now pressed, stop decrementing
				}
			}
		}
		else
		{
			sec_dec_button = 1; //Reset button to original released state, to decrement again next time.
		}


		if(! (PINB & (1<<PB4)))
		{
			_delay_ms(30);      //overcome de-bounce

			if(! (PINB & (1<<PB4)))
			{
				if(min_inc_button)
				{
					min_ones++;

					if(min_ones >= 10)
					{
						min_ones = 0;
						min_tens++;
					}

					if (min_tens >= 6)
					{
						min_tens = 0;
					}

					min_inc_button = 0;
				}
			}
		}
		else
		{
			min_inc_button = 1;
		}

		if(! (PINB & (1<<PB3)))
		{
			_delay_ms(30);    //overcome de-bounce


			if(! (PINB & (1<<PB3)))
			{
				if(min_dec_button)  //Only decrement if the button was previously released
				{

					if(min_ones != 0)
					{
						min_ones--;
					}
					else if(min_tens != 0)
					{
						min_ones = 9;
						min_tens--;
					}
					else
					{
						//to prevent timer from decrementing after 00
					}

					min_dec_button = 0;   //Button is now pressed, stop decrementing
				}
			}
		}
		else
		{
			min_dec_button = 1; //Reset button to original released state, to decrement again next time.
		}


		if(! (PINB & (1<<PB1)))
		{
			_delay_ms(30);       //overcome de-bounce

			if(! (PINB & (1<<PB1)))
			{
				if(hours_inc_button)
				{
					hours_ones++;

					if(hours_ones >= 10)
					{
						hours_ones = 0;
						hours_tens++;
					}

					if (hours_tens >= 6)
					{
						hours_tens = 0;
					}

					hours_inc_button = 0;
				}
			}
		}
		else
		{
			hours_inc_button = 1;
		}


		if(! (PINB & (1<<PB0)))
		{
			_delay_ms(30);   //overcome de-bounce

			if(! (PINB & (1<<PB0)))
			{
				if(hours_dec_button)  //Only decrement if the button was previously released
				{

					if(hours_ones != 0)
					{
						hours_ones--;
					}
					else if(hours_tens != 0)
					{
						hours_ones = 9;
						hours_tens--;
					}
					else
					{
						//to prevent timer from decrementing after 00
					}

					hours_dec_button = 0;   //Button is now pressed, stop decrementing
				}
			}
		}
		else
		{
			hours_dec_button = 1; //Reset button to original released state, to decrement again next time.
		}

	} //End while Loop

}



