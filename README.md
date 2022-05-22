# Flight radar

Визуализатор авиа-радара для бесконечного случайного расписания прилетов и отбытий воздушных суден из аэродрома. Позволяет регулировать скорость обновлений картинки и максимальное количество суден, располагающихся в пределах аэродрома одновременно.

![](demo.gif)

## Требования к расписанию:
* Радаром опознаются перемещения прилетающих воздушных суден (синие точки), отбывающих (зеленые точки), а также специальной техники (желтые точки)
* На взлетно-посадочной полосе может находиться не более одного судна в единицу времени
* Каждая рулежная дорожка (съезд/подъезд к ВПП) может быть использована для перевижения не более одного судна в единицу времени
* Не должно возникать голодания ожидания освобождения дорожек/ВПП для конкретного воздушного судна
* Передвижение специальной техники не должно накладывать дополнительных ограничений на передвижение воздушных суден

## Todo:
* Параллельно вычислять следующие позиции суден
