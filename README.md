# PROGETTO PENDOLO INVERSO

## CONTROLLO DI UN PENDOLO INVERSO MONTATO SU UN CARRELLO

Il progetto consiste nella realizzazione di un pendolo inverso montato su di un carrello, lo scopo è quello di 
mantenere in equilibrio il pendolo sfruttando diverse tecniche di controllo. \
Il pendolo è composto da un'asta libera di cadere lungo l'asse di movimento del carrello, il quale può 
spostarsi avanti e indietro con l’utilizzo di quattro motori montati su di esso. \
L’asta è fissata ad un encoder per poterne determinare la posizione angolare, inoltre il sistema è dotato 
anche di un sonar che permette di misurare la distanza del carrello da un eventuale ostacolo o una parete, 
in questo modo è possibile determinare la posizione relativa del carrello rispetto ad un elemento fisso
(come ad esempio una parete), e quindi, in una seconda fase applicare un controllo anche sulla posizione 
del carrello. 

## Schema del sistema carrello-pendolo
<a href="1"><img src="https://github.com/DavideOlivieri/Progetto-pendolo-inverso-controllo-PID-/assets/83114199/4956e1ca-0d9b-49f6-ba42-27272d0bd540" height="400" width="500" ></a> 
<a href="1"><img src="https://github.com/DavideOlivieri/Progetto-pendolo-inverso-controllo-PID-/assets/83114199/fa4ceac8-003e-4e9b-b4f7-05e8eab0e83a" height="400" width="500" ></a> 

***

## Software 

La scheda STM32 è stata programmata in C tramite l’ambiente di sviluppo STM32CubeIDE (STM32CubeIDE
v1.10.1, STM32CubeMX v6.8.0). \
Il software è stato realizzato in modo da poterlo adattare molto semplicemente all’utilizzo con le differenti 
tecniche di controllo applicate. \
In particolare, il software è stato sviluppato implementando separatamente la gestione delle componenti 
hardware (motori, sonar encoder, ecc…) dalla gestione del controllo. \
In questo modo per applicare differenti tecniche di controllo è necessario modificare soltanto la parte che 
si occupa del calcolo dell’input di controllo da fornire ai motori. \
L’esecuzione del software è governata da un timer che scandisce l’intervallo temporale nel quale effettuare 
le letture dei sensori (encoder e sonar), calcolare la risposta necessaria a correggere la posizione dell’asta 
e/o la posizione del robot, e infine applicare questa risposta azionando opportunamente i motori. \
Inoltre, per renderne pratico l’utilizzo, è stato configurato il pulsante disponibile sulla scheda STM32
(pulsante BLU) per abilitare i motori, in questo modo il carrello anche se acceso e alimentato inizia a 
muoversi solo dopo la pressione del pulsante in questione, premendo ancora il pulsante si possono 
arrestare nuovamente i motori. 

## Controllo PID

Il controllo Proporzionale-Integrale-Derivativo (PID) è uno dei sistemi di controllo in retroazione più diffusi e 
utilizzati grazie alla sua flessibilità e al fatto che può essere applicato ad una vasta gamma di problemi di 
controllo senza conoscere il modello del sistema sul quale lo si sta applicando. \
Un controllore PID permette di controllare una variabile di un sistema a partire da un valore in ingresso e 
un valore di riferimento (Setpoint), in particolare per determinare l’output del controllore PID viene usato 
l’errore del valore in ingresso fornito al PID rispetto il Setpoint desiderato. \
In particolare, il risultato fornito da un controllo PID è calcolato come la somma di tre termini:
- Termine proporzionale: P
- Termine integrale: I
- Termine derivativo: D
Combinando in maniera differente i singoli parametri proporzionale, integrale e derivativo, si ottengono 
diverse strutture del controllore PID, in particolare quella utilizzata in questa applicazione è la struttura 
Parallela, in cui i termini P, I e D sono sommati tra loro: \
𝑢(𝑡) = 𝑢𝑝(𝑡) + 𝑢𝑖(𝑡) + 𝑢𝑑(𝑡)


