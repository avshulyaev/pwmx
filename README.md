### PWMX
Библиотека PWMX позволяет иницилизировать переферию контроллера STM32F429 для 
генерации PWM сигнала, с заданной скважностью.
Данная библиотека написана для работы с МК STM32F429 и содержит платформо-зависимые элементы.


### Пример использования:
```
#include PWMX.h

void main(void){

// Провести инициализацию HAL библиотеки
// и провести настройку RCC 
// для деталей см. pwm_example 

// Инициализация библиотеки и всей необходимой переферии
if (PWM_Init(5, 2, PA10, PA11) = OK){

// Выбор нужной скважности
pwm_out_set(i, 50);
// Выгрузка библиотеки
PWM_DeInit();
}

}
```

### Как использовать
Данная библиотека распространяется в исходниках и для ее использования необходимо 
добавить PWMX.c к проекту и прописать в путь к инклудам путь до PWMX.h и PWMX_private.h
