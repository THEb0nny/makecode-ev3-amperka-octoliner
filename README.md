# Amperka Octoliner Extension for MakeCode EV3

Расширение для среды **MakeCode for LEGO® MINDSTORMS® EV3 (BrickCode)**, добавляющее поддержку 8-канального аналогового ИК-датчика линии **«Октолайнер» (серия Zelo / Troyka)** от Амперки.

## Подключение датчика к EV3

Датчик подключается к любому из входных портов EV3 (**1, 2, 3 или 4**) по шине I²C:

| Контакт кабеля EV3 | Цвет жилы | Назначение | Контакт на Октолайнере |
| :--- | :--- | :--- | :--- |
| **Пин 1** | Белый | AN (Аналоговый) | *Не подключается* |
| **Пин 2** | Черный | GND (Земля) | **G (GND)** |
| **Пин 3** | Красный | GND (Земля) | *Не подключается* |
| **Пин 4** | Зеленый | VCC (+4.3В .. +5В) | **V (VCC)** |
| **Пин 5** | Желтый | SCL (I²C Clock) | **SCL** |
| **Пин 6** | Синий | SDA (I²C Data) | **SDA** |

> Дополнительные подтягивающие резисторы не требуются (в блоке EV3 уже установлены pull-up резисторы 4.7 кОм на линиях SDA и SCL).

## Использование расширения

1. Откройте [https://brickcode.org](https://brickcode.org) (или MakeCode EV3).
2. Создайте новый проект или откройте существующий.
3. Перейдите в меню **Расширения** (Extensions) под значком шестеренки.
4. Введите URL репозитория: `https://github.com/theb0nny/makecode-ev3-amperka-octoliner` и нажмите Enter.

## Основные блоки и методы

### Положение линии
```typescript
let pos = sensors.octoliner1.linePosition();
```
Возвращает отклонение линии от центра датчика в диапазоне от **`-100`** (крайний левый) до **`+100`** (крайний правый). Если линия ровно по центру — возвращается **`0`**.

### Проверка наличия линии
```typescript
if (sensors.octoliner1.isOnLine()) {
    // Датчик видит линию
}
```

### Проверка перекрестка
```typescript
if (sensors.octoliner1.isIntersection()) {
    // Все 8 датчиков видят черное
}
```

### Битовая маска
```typescript
let mask = sensors.octoliner1.readBitmask();
```
Возвращает 8-битную маску состояния каналов (1 — черное/линия, 0 — белое/фон). Бит 7 — канал 0, Бит 0 — канал 7.

### Чтение отдельного канала
```typescript
let val = sensors.octoliner1.readChannel(OctolinerChannel.Channel0);
```
Считывает сырое значение АЦП (от 0 до 4095).

### Настройка чувствительности
```typescript
sensors.octoliner1.setSensitivity(208);
```
Задает чувствительность фотоприемников от 0 до 255 (по умолчанию 208).

## Пример: П-регулятор движения по линии

```typescript
const baseSpeed = 40;
const kp = 0.6;

forever(function () {
    let error = sensors.octoliner1.linePosition();
    let turn = error * kp;
    motors.largeBC.tank(baseSpeed + turn, baseSpeed - turn);
});
```

#### Metadata (used for search, rendering)

* for PXT/ev3
* supportedTargets: ev3
