# ble_app_moustache

## **Skraćeno**

Zadatak se sastoji od jedne usluge sa tri karakteristike:
- prva nam služi da svakom perifernom uređaju dodijelimo boju (tj. jedinstvenu šifru boje)
- druga nam služi da odredimo koji je igrač prvi stisnuo gumb
- treća nam služi da proglasimo tko je prvi stisnuo gumb te da periferni uređaj zna je li film zaustavljen


## *Dvije faze zadatka*

Logiku zadatka bi podijelio na dvije faze:

#### Faza inicijalizacije sustava
- spajanje perifernih i centralnog uređaja tijekom koje se razmjenuju podaci nužni za rješavanje našeg problema

#### Faza rješavanje zadatka
- korištenje prethodno razmijenjenih podataka kako bi se uspješno izvele sve funkcionalnosti zadatka

## *Detaljniji opis karakteristika*

#### Prva
- ima mogućnosti čitanja i obavijesti (notify). 
- ima CCCD desktriptor
- koristi se isključivo u prvoj fazi zadatka

#### Druga 
- ima mogućnost pisanja. 
- koristi se iskjučivo u drugoj fazi zadatka

#### Treća ili ID karakteristika
- ima mogućnosti čitanja i obavijesti
- ima CCCD deskriptor
- služi kao binarni semafor
- označava periferni uređaj koji je prvi stisnuo tipku
- koristi se isključivo u drugoj fazi zadatka

## *Detaljniji opis toka zadatka*

Sada ću detaljnije opisati svaku od faza na temelju primjera koji će se sastojati od jednog centralnog 
i dva periferna uređaja:

#### Prva faza
- centralni uređaj skanira pakete koji su slani od strane dva periferna uređaja
- nakon što se centralni uređaj spojio s perifernim:
  - periferni uređaj postavlja CCCD deskriptor prve i ID karakteristike na 1
  - centrala odgovara tako što postavlja prvu karakteristiku na zadanu vrijednost koja predstavlja jedinstvenu boju perifernog uređaja
  - centrala potom obaviještava periferni o promijeni vrijednosti
  -  periferni uređaj sprema lokalnu šifru boje
  - centrala potom postavlja vrijednost prve na 0
  - centala nastavlja skenirati ostale uređaje
- nakon spajanja s drugim perifernim uređajem centrala ima isti postupak kao s prvim perifernim samo naravno s drukčijim vrijednostima
- centrala nakon određenog broja spojenih uređaja prestaje skenirati

#### Druga faza

- film se vrti, u jednom trenutku igrač hoće zaustaviti film
- igrač stisne gumb:
  - periferni uređaj pregledava lokalno spremljenu kopiju ID karakteristike 
    - ako je ID karakteristika nula ili jednaka vlastitom ID-u pokušat će upisati svoj id boje u drugu karakteristiku
    - ako je različita od nule, periferni ne radi ništa
- kada paket dođe do centralnog on provjerava je li druga karakteristika 0
  - ako je 0 :
    -  lokalno sprema kopiju ID karakteristike 
    -  drugu karakteristiku postavlja na određen ID 
    -  obaviještava sve periferije
  - ako nije 0 :
    - centralni ne radi ništa
- svaka periferije po primitku provjerava je li vrijednost jednaka vlastitoj :
  - ako je pale se lampice
  - ako nije ništa se ne događa
- centralni je u stanju pripravnosti 
- čeka trenutak kada će periferni uređaj sa ID-om *ID karakteristike* pokušati pisati na drugu karakteristiku
- kada se to dogodi :
  -  ID karakteristika se postavlja na nulu
  -  obaviještavaju se svi 
  -  film nastavlja vrtiti
