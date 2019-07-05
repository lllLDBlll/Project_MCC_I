? addr : endere¸co do dispositivo que se deseja enviar uma mensagem.
? cmd: comando que se deseja enviar, geralmente escrita ou leitura.
? reg: qual registrador do dispositivo deseja-se escrever ou ler.
? data: dado que se deseja escrever no registrador ou n´umero de registradores que se deseja
ler.
? crc: verifica¸c˜ao c´iclica de redundˆancia.

addr 	cmd 	reg 	data 	crc
1byte 	1byte 	2byte	2byte	2byte
		big en  big en  big en


addr - 0x15
comando de escrita - 0x01
comando de leitura - 0x02

Registradores disponiveis: 
estado: estado da conex˜ao WiFi.
?atuador x: valores provenientes da “Internet” que podem acionar algum atuador
?sensor x: valores escritos pelo ATmega328P que ser˜ao enviados pelo m´odulo `a “Internet”

Read Only
State 	  - 0x00
Atuador_0 - 0x01
Atuador_1 - 0x02
Atuador_2 - 0x03
Atuador_3 - 0x04

Write
Sensor_0 - 0x05
Sensor_1 - 0x06
Sensor_2 - 0x07
Sensor_3 - 0x08
    2 bytes

Tem código de erros

README.md 
CONTRIBUTORS.md 
Makefile






