/*
*Iann Grijalva-23055
* Universidad del Valle de Guatemala
* IE2023: Programación de Microcontroladores
* contador 8 bits con contador hex y alarama
*
* Descripción:contador de 8 bits en C con contador hex y alarma
*
*
* Hardware: ATMega328P
*/


#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdlib.h> // para abs()

// definiciones de pines para los botones
#define PIN_BOTON_INCREMENTO PB0  // pin para el botón de incremento conectado a PB0
#define PIN_BOTON_DECREMENTO PB1  // pin para el botón de decremento conectado a PB1

// pines de control para los displays
#define PIN_DISPLAY_1 PC4  // pin para controlar el primer display 
#define PIN_DISPLAY_2 PC5  // pin para controlar el segundo display 

// pin para el LED indicador de comparación
#define PIN_LED_COMPARACION PD7


// estos valores representan los patrones para mostrar 0-9 y A-F
const uint8_t PATRONES_7SEG[16] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
    0b01110111, // A
    0b01111100, // b
    0b00111001, // C
    0b01011110, // d
    0b01111001, // E
    0b01110001  // F
};

// variables globales
volatile uint8_t display_actual = 0;   // control de multiplexación (0=primer display, 1=segundo display)
volatile uint8_t valor_adc_hex = 0;    // valor del ADC mapeado a 0-255 (para displays hex)
uint8_t contador = 0;                  // valor del contador

/**
 * función para inicializar el Timer0 para multiplexación
 * configura el timer para generar interrupciones periódicas
 */
void inicializar_timer0(void) {
    // configurar Timer0 en modo CTC 
    TCCR0A = (1 << WGM01);
    
    // establecer el preescalador a 64
    TCCR0B = (1 << CS01) | (1 << CS00);
    
    // valor de comparación para generar interrupciones aproximadamente cada 1ms
    OCR0A = 249;
    
    // habilitar interrupción por comparación
    TIMSK0 = (1 << OCIE0A);
}

/**
 * maneja la multiplexación de los displays
 */
ISR(TIMER0_COMPA_vect) {
    // Guardar el estado actual del bit PD7 
    uint8_t estado_led_comparacion = PORTD & (1 << PIN_LED_COMPARACION);
    
    // desactivar ambos displays
    PORTC &= ~((1 << PIN_DISPLAY_1) | (1 << PIN_DISPLAY_2));
    
    // alternar entre los displays
    if (display_actual == 0) {
        // mostrar dígito menos significativo (0-F)
        // Mantener el bit 7 (PIN_LED_COMPARACION) sin cambios
        PORTD = (PATRONES_7SEG[valor_adc_hex & 0x0F] & 0x7F) | estado_led_comparacion;
        PORTC |= (1 << PIN_DISPLAY_1);
        display_actual = 1;
    } else {
        // mostrar dígito más significativo (0-F)
        // Mantener el bit 7 (PIN_LED_COMPARACION) sin cambios
        PORTD = (PATRONES_7SEG[(valor_adc_hex >> 4) & 0x0F] & 0x7F) | estado_led_comparacion;
        PORTC |= (1 << PIN_DISPLAY_2);
        display_actual = 0;
    }
}

/**
 * función para inicializar los puertos del microcontrolador
 */
void inicializar_puertos(void) {
    // configurar PB0 y PB1 como entradas con resistencias pull-up internas activadas
    DDRB &= ~((1 << PIN_BOTON_INCREMENTO) | (1 << PIN_BOTON_DECREMENTO));
    PORTB |= (1 << PIN_BOTON_INCREMENTO) | (1 << PIN_BOTON_DECREMENTO);
    
    // configurar PB2, PB3, PB4, PB5 como salidas para mostrar parte del valor del contador
    DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5);
    
    // configurar PC0, PC1, PC2, PC3 como salidas para parte del valor del contador
    // y PC4, PC5 como salidas para controlar los displays
    DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PIN_DISPLAY_1) | (1 << PIN_DISPLAY_2);
    
    // configurar PD0-PD6 como salidas para los segmentos del display (A-G)
    // y PD7 como salida para el LED de comparación
    DDRD |= 0b11111111; // bits 0-7 como salidas
    
    // inicializar todas las salidas en nivel bajo (0 lógico)
    PORTB &= ~((1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5));
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PIN_DISPLAY_1) | (1 << PIN_DISPLAY_2));
    PORTD &= ~0b11111111;
}

/**
 * función para inicializar el ADC
 */
void inicializar_adc(void) {
    // seleccionar AVCC como referencia de voltaje
    ADMUX = (1 << REFS0);
    
    // seleccionar el canal ADC6
    ADMUX |= 0x06;  // 0b0110
    
    // habilitar ADC, establecer prescaler a 128 para obtener una frecuencia de muestreo adecuada
    // (16MHz/128 = 125kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

/**
 * función para leer el valor del ADC con filtro
 */
uint16_t leer_adc(void) {
    // número de muestras para promediar
    const uint8_t NUM_MUESTRAS = 8;
    uint32_t suma = 0;
    
    // tomar varias muestras y sumarlas
    for (uint8_t i = 0; i < NUM_MUESTRAS; i++) {
        // iniciar conversión
        ADCSRA |= (1 << ADSC);
        
        // esperar hasta que la conversión termine
        while (ADCSRA & (1 << ADSC));
        
        // sumar la lectura
        suma += ADC;
        
        //retraso entre muestras
        _delay_us(100);
    }
    
    // calcular el promedio
    return (uint16_t)(suma / NUM_MUESTRAS);
}

/**
 * función para verificar si un botón está presionado 
 * 1 si el botón está presionado, 0 en caso contrario
 */
uint8_t esta_boton_presionado(uint8_t pin_boton) {
    // verificar si el botón está presionado 
    if (!(PINB & (1 << pin_boton))) {
        return 1; // botón presionado
    }
    return 0; // botón no presionado
}

/**
 * función para actualizar la visualización del valor del contador en los pines de salida
 * cada bit del valor del contador se mapea a un pin específico
 */
void actualizar_visualizacion_contador(uint8_t valor_contador) {
    // bit 0 (bit menos significativo) - PB5
    if (valor_contador & 0x01) {
        PORTB |= (1 << PB5);
    } else {
        PORTB &= ~(1 << PB5);
    }
    
    // bit 1 - PC0
    if (valor_contador & 0x02) {
        PORTC |= (1 << PC0);
    } else {
        PORTC &= ~(1 << PC0);
    }
    
    // bit 2 - PC1
    if (valor_contador & 0x04) {
        PORTC |= (1 << PC1);
    } else {
        PORTC &= ~(1 << PC1);
    }
    
    // bit 3 - PC2
    if (valor_contador & 0x08) {
        PORTC |= (1 << PC2);
    } else {
        PORTC &= ~(1 << PC2);
    }
    
    // bit 4 - PC3
    if (valor_contador & 0x10) {
        PORTC |= (1 << PC3);
    } else {
        PORTC &= ~(1 << PC3);
    }
    
    // bit 5 - PB4
    if (valor_contador & 0x20) {
        PORTB |= (1 << PB4);
    } else {
        PORTB &= ~(1 << PB4);
    }
    
    // bit 6 - PB3
    if (valor_contador & 0x40) {
        PORTB |= (1 << PB3);
    } else {
        PORTB &= ~(1 << PB3);
    }
    
    // bit 7 (bit más significativo) - PB2
    if (valor_contador & 0x80) {
        PORTB |= (1 << PB2);
    } else {
        PORTB &= ~(1 << PB2);
    }
}

/**
 * función para comparar el valor del contador binario con el contador hexadecimal
 * y actualizar el LED indicador en PD7
 */
void actualizar_led_comparacion(uint8_t valor_contador_binario, uint8_t valor_contador_hex) {
    // Asegurar que las interrupciones estén deshabilitadas durante la actualización
    cli();
    
    // comparar los valores y actualizar el LED en PD7
    if (valor_contador_hex > valor_contador_binario) {
        // encender el LED si el contador hexadecimal es mayor
        PORTD |= (1 << PIN_LED_COMPARACION);
    } else {
        // apagar el LED si el contador hexadecimal es menor o igual
        PORTD &= ~(1 << PIN_LED_COMPARACION);
    }
    
    // Rehabilitar interrupciones
    sei();
}

/**
 * función principal del programa
 */
int main(void) {
    // declaración e inicialización de variables
    uint8_t incremento_presionado = 0;   // flag para evitar incrementos múltiples
    uint8_t decremento_presionado = 0;   // flag para evitar decrementos múltiples
    uint16_t valor_adc_actual = 0;       // valor actual del ADC 
    
    // inicializar configuración de los puertos ADC y Timer
    inicializar_puertos();
    inicializar_adc();
    inicializar_timer0();
    
    // mostrar el valor inicial del contador 
    actualizar_visualizacion_contador(contador);
    
    // inicializar el LED de comparación antes de habilitar las interrupciones
    if (valor_adc_hex > contador) {
        PORTD |= (1 << PIN_LED_COMPARACION);
    } else {
        PORTD &= ~(1 << PIN_LED_COMPARACION);
    }
    
    // habilitar interrupciones globales
    sei();
    
    // bucle infinito principal del programa
    while (1) {
        // leer el valor del ADC
        valor_adc_actual = leer_adc();
        
        //  potenciómetro de 100K
        static uint32_t valor_filtrado = 0;
        static uint8_t contador_estabilidad = 0;
        
        // mapear el valor del ADC con compensación de rango específica
        uint16_t valor_compensado;
        
        // ajuste específico para potenciómetro de 100K para capturar el rango completo incluyendo FF
        // si el valor es muy bajo menor a 20 forzarlo a 0
        if (valor_adc_actual < 20) {
            valor_compensado = 0;
        }
        // si el valor es muy alto  forzarlo a 1023 para garantizar que llegue a FF
        else if (valor_adc_actual > 1000) {
            valor_compensado = 1023;
        }
        // de lo contrario, hacer un mapeo más fino del rango efectivo (20-1000) a (0-1023)
        else {
            // aplicar un factor de escala ligeramente mayor para asegurar que llegue a 1023
            valor_compensado = ((uint32_t)(valor_adc_actual - 20) * 1030UL) / (1000UL - 20UL);
            // saturar a 1023 si el cálculo resultara en un valor mayor
            if (valor_compensado > 1023) {
                valor_compensado = 1023;
            }
        }
        
      
        // dar más peso al valor anterior para una mejor estabilidad
        valor_filtrado = (valor_filtrado * 7 + valor_compensado) / 8;
        
        // cuando el valor está cerca del máximo, ayudar a que llegue a FF
        if (valor_compensado > 1020) {
            valor_filtrado = 1023; // forzar al máximo para llegar a FF
        }
        
        // convertir a rango 0-255 para representación hexadecimal
        uint8_t nuevo_valor_hex = (uint8_t)((valor_filtrado * 255UL) / 1023UL);
        
        // solo actualizar cuando el valor sea realmente diferente
        if (nuevo_valor_hex != valor_adc_hex) {
            // agregar contador para evitar actualización si es solo un pico 
            contador_estabilidad++;
            if (contador_estabilidad >= 3) { // el valor debe ser estable por al menos 3 ciclos
                valor_adc_hex = nuevo_valor_hex;
                contador_estabilidad = 0;
                
                // actualizar el LED de comparación al cambiar el valor del ADC
                actualizar_led_comparacion(contador, valor_adc_hex);
            }
        } else {
            contador_estabilidad = 0; // resetear contador si el valor es igual
        }
        
        // verificar botones para el contador
        if (esta_boton_presionado(PIN_BOTON_INCREMENTO)) {
            if (!incremento_presionado) {
                contador++;
                actualizar_visualizacion_contador(contador);
                incremento_presionado = 1;
                
                // actualizar el LED de comparación al cambiar el contador
                actualizar_led_comparacion(contador, valor_adc_hex);
            }
        } else {
            incremento_presionado = 0;
        }
        
        if (esta_boton_presionado(PIN_BOTON_DECREMENTO)) {
            if (!decremento_presionado) {
                contador--;
                actualizar_visualizacion_contador(contador);
                decremento_presionado = 1;
                
                // actualizar el LED de comparación al cambiar el contador
                actualizar_led_comparacion(contador, valor_adc_hex);
            }
        } else {
            decremento_presionado = 0;
        }
        
        
		
        _delay_ms(10);
    }
    
    return 0;
}

