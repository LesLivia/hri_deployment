#!/usr/bin/env python
import time
import os
import sys
import rospy_utils.hrirosnode as hriros
import vrep_utils.vrep as vrep
import agents.navigation as nav
from threading import Thread
from multiprocessing import Pool
from agents.mobilerobot import *   #MobileRobot, start_reading_data_robs, follow_position_rob
from agents.human import Human, start_reading_data, follow_position, follow_fatigue, FatigueProfile  # posso importare tutto ma cosi è piu comodo quando chiamo le funzioni
from agents.coordinates import Point
from agents.orchestrator import Orchestrator, OpChk  # lo stesso
from agents.mission import *  # * vuol dire importo tutto

import numpy as np

print('Launching application...')

vrep_sim = vrep.connect(19997)
vrep.start_sim(vrep_sim)


# HUMANS

bill = Human(1, 10, FatigueProfile.YOUNG_SICK, 1 , character.YOUNG_SICK )  # giovane ammalato
carl = Human(2, 10, FatigueProfile.YOUNG_HEALTHY, 1 , character.DOCTOR)  # dottore
david = Human(3, 10, FatigueProfile.YOUNG_HEALTHY, 1 , character.YOUNG_HEALTHY)   # giovane sano
eve = Human(4, 6, FatigueProfile.ELDERLY_SICK, 1 , character.ELDERLY_SICK)	# vecchio ammalato
frank = Human(5, 5, FatigueProfile.ELDERLY_HEALTHY, 1 , character.ELDERLY_HEALTHY)	# vecchio sano
giuly = Human(6, 4, FatigueProfile.YOUNG_HEALTHY, 1 , character.NURSE)	# infermiera
hank = Human(7, 15, FatigueProfile.YOUNG_HEALTHY, 1 , character.NURSE)	#infermiere




humans = [bill, carl, david,eve,frank,giuly,hank]

# ROBOTS

rob1 = MobileRobot(1, 8.0, 5.0)
rob2 = MobileRobot(2, 8.0, 5.0)
rob3 = MobileRobot(3, 8.0, 5.0)

robs=[rob1,rob2,rob3]
#robs=[rob1]

# DESTINATIONS

#dest = [Point(24.0, 10.5)]
dest1 = Point(12,9)
dest2 = Point(20,12)
dest3 = Point(24.0, 10.5)
dest4 = Point(27,12)
dest5 = Point(27,9)



dests1 = [dest4]
dests2 = [dest2,dest1,dest2]
dests3 = [dest2,dest3]


# PATTERNS

patterns1 = [Pattern.HUM_FOLLOWER]
#patterns1 = [Pattern.HUM_LEADER]
#patterns1 = [Pattern.HUM_FOLLOWER]
#patterns1 = [Pattern.HUM_INTER]
patterns2=[Pattern.HUM_INTER,Pattern.HUM_RECIPIENT,Pattern.HUM_FOLLOWER]
patterns3 = [Pattern.HUM_LEADER, Pattern.HUM_FOLLOWER]



# MISSIONS

mission1 = Mission(patterns1, dests1, 1)
mission2 = Mission(patterns2, dests2, 2)
mission3 = Mission(patterns3, dests3, 3)

#total_missions=[mission1,mission2,mission3]
total_missions=[mission1]
#hums_to_serve=[[frank],[eve,eve,eve],[hank,bill]]
hums_to_serve=[[eve]]

# print('inizio prova')
# humsss=[carl ,  david , bill , eve]
# humsss.sort(key=Human.character.get_importance())
#
# print('fine prova')

ottimizzazione = 1
# ottimizzazione = 0 --> non c è        ( 0 o qualsiasi altro numero )
# ottimizzazione = 1 --> locale_per_hum. ( ordinamento_hums = 1 / 2 ) ( metodo_assegnazione_rob = 1 / 2 / 3)
# ottimizzazione = 2 --> locale_per_durata_mission. ( metodo_assegnazione_rob = 2 / 3 )
# ottimizzazione = 3 --> globale per distanze (voglio minimizzare il tempo ma non garantisco la non sync). ( ordinamento_hums  per cardinalita) ( successivamente : metodo_assegnazione_rob = 1 )
# ottimizzazione = 4 --> globale per distanze ma con una certa confidenza sul successo.( successivamente : metodo_assegnazione_rob = 3 )
# ottimizzazione = 5 --> globale con lo scopo che la maggior parte delle missioni arrivi al successo con una certa confidenza. ( metodo_assegnazione_rob = 4 )
# le ottimizzazioni locali restituiscono l ordine con cui le missioni vengono analizzate una alla volta
# le ottimizzazioni globali restituiscono un vettore dove sappiamo gia quali robots sono associati a quali missions ( non in tutti i casi)


ordine_crescente = True
# in ottimizzazione = 2 : normalmente è per ordine decrescente. se voglio ordine crescente metto ordine_crescente = True

ordinamento_hums = 2
# ordinamento_hums = 1 --> guardo importanza primo hum tra quelli che devo servire nella mission
# ordinamento_hums = 2 --> guardo importanza tutti hum


metodo_assegnazione_rob = 1
# metodo_assegnazione_rob = 1 --> rob piu vicino
# metodo_assegnazione_rob = 2 --> rob piu carico
# metodo_assegnazione_rob = 3 --> rob piu vivino con una carica accettabile. altrimenti metodo_assegnazione_rob = 2
# metodo_assegnazione_rob = 4 --> rob piu scarico con una carica accettabile. altrimenti metodo_assegnazione_rob = 2




# funzioni poi da trasferire nell orch generale

def all_hums_free(hums: List[Human]):
    for hum in hums:
        if hum.free==False:
            return False
    return True

def need_sync(orch_list: List[Orchestrator]):
    if len(orch_list)==0:
        return False
    for orch in orch_list:
        if orch.need_sync==True:
            return True
    return False

def sync(orch: Orchestrator, rob: MobileRobot, orch_id: int):
    orch.rob=rob
    orch.opchk.rob=rob
    orch.opchk.orch_id=orch_id
    Thread(target=orch.run_mission).start()

def rob_go_to_hum_and_sync(orch: Orchestrator, rob: MobileRobot, orch_id: int):
    rob_go_to_hum(rob, orch.opchk.humans[orch.opchk.currH])
    sync(orch, rob, orch_id)

def sync_in_rec_stages2(orch: Orchestrator, rob: MobileRobot, orch_id: int):
    robs_go_to(orch.rob, rob, orch.opchk.humans[orch.opchk.currH])
    orch.sync_in_rec_stage2=True
    while orch.proceed_in_main == False:
        pass
    orch.proceed_in_main = False
    sync(orch,rob,orch_id)

def inverti_ordine_mis( missions: List[Mission]):
    new_missions=[]
    while len(missions) != 0:
        new_missions.append(missions[-1])
        missions.pop(-1)
    return  new_missions

def inverti_ordine_humss( humss: List[List[Human]]):
    new_humss=[]
    while len(humss) != 0:
        new_humss.append(humss[-1])
        humss.pop(-1)
    return new_humss

def ottimizza_globale_batteria(robs: List[MobileRobot] ,  missions: List[Mission] , humss: List[List[Human]]):
    # se gli passo le mission in ordine crescente di durata, faccio piu assegnazioni possibili e poi
    # superata una certa soglia di durata / batteria-necessaria esco dal ciclo perche so che anche con
    # le altre successive missioni non posso fare piu assegnazioni
    assegnazione_rob_hum=[]
    freerobs = free_robs(robs)
    hums_liberi = []
    missions_libere=[]
    for hums in humss:
        if all_hums_free(hums):
            hums_liberi.append(hums)
            for hum in hums:
                hum.free = False
            missions_libere.append(missions[humss.index(hums)])
    # li rimetto tutti allo stato iniziale. poi prima di far partire l orch li metto occupati, senno potrei avere problemi se successivamente
    # esco dalla matrice senza che questa sia nulla perche mi rimangono degli hum occupati mentre in realta non sono stati assegnati e quindi
    # sono liberi
    for hums in humss:
        for hum in hums:
            hum.free = True
    cariche_minime_accettabili=[]
    for mis in missions_libere:
        cariche_minime_accettabili.append(carica_minima_mission(mis))
    while len(hums_liberi) != 0 or len(freerobs) != 0:
        rob_chosen = scegli_rob(freerobs , hums_liberi[0][0] , metodo_assegnazione_rob , cariche_minime_accettabili[0])
        if rob_chosen == None :
            break
        assegnazione_rob_hum.append( [rob_chosen , hums_liberi[0]])
        cariche_minime_accettabili.pop(0)
        hums_liberi.pop(0)
        freerobs.remove(rob_chosen)
    return assegnazione_rob_hum

def ottimizza_globale_batteria_dist( robs: List[MobileRobot] , missions: List[Mission] , humss: List[List[Human]]):  # passare copie

    distanza_infinita= 100000
    assegnazione_rob_hum=[]
    freerobs = free_robs(robs)
    hums_liberi = []
    missions_libere=[]
    for hums in humss:
        if all_hums_free(hums):
            hums_liberi.append(hums)
            for hum in hums:
                hum.free = False
            missions_libere.append(missions[humss.index(hums)])
    # se successivamente esco dalla matrice senza che questa sia vuota, sto lasciando degli umani occupati anche se in realta
    # non sono stati assegnati. per questo adesso li rimetto tutti liberi, e poi prima di far partire l orch li metto occupati
    # missione per missione. tanto adesso ho gia creato i vettori hums_liberi e missions_libere di riferimento
    for hums in humss:
        for hum in hums:
            hum.free = True
    punti_rob=[]
    for rob in freerobs:
        pos=rob.get_position()
        point_rob= Point(pos.x , pos.y)
        punti_rob.append(point_rob)
    punti_hum=[]
    for hums in hums_liberi:
        if missions_libere[hums_liberi.index(hums)].p[0] == Pattern.HUM_RECIPIENT or missions_libere[hums_liberi.index(hums)].p[0] == Pattern.HUM_INTER:
            # con human recipient ci sono 2 stage, non ha senso calcolare all inizio la distanza hum-rob perche quella è la stage 2
            # calcolo la distanza a cui deve arrivare prima di entrare in stage 2
            # poi la metto in point hum anche se non è un punto dell umano
            punti_hum.append(missions_libere[hums_liberi.index(hums)].dest[0])
            print(' sono qui')
            continue
        pos=hums[0].get_position()
        point_hum= Point(pos.x , pos.y)
        punti_hum.append(point_hum)
    # adesso creo la matrice
    matrice_distanze=[]
    for point_hum in punti_hum:
        distanza_hums_rob=[]        # in realta è distaza hum robs. guardare se la riuso dopo e se mi sono confuso da qualche parte
        for point_rob in punti_rob:
            distance= point_hum.distance_from(point_rob)
            distanza_hums_rob.append(distance)
        matrice_distanze.append(distanza_hums_rob)
    print(' matrice distanze = ', matrice_distanze)
    # righe ( = vettori) = distanza primo hum con i vari rob
    # colonne = distanza rob con i vari hum
    for riga in matrice_distanze:
        carica_minima_accettabile = carica_minima_mission(missions_libere[matrice_distanze.index(riga)])
        for rob in freerobs:
            if rob.get_charge() < carica_minima_accettabile:
                riga[freerobs.index(rob)] = distanza_infinita
    print(' matrice distanze con possibili distanze infinite = ', matrice_distanze)
    while len(matrice_distanze) != 0 and len(matrice_distanze[0]) != 0:
        print(' len matrice distanze = ', len(matrice_distanze))
        temp=[]
        indici_colonna_min=[]
        for riga in matrice_distanze:
            indice_colonna_min=riga.index(min(riga))
            indici_colonna_min.append(indice_colonna_min)
            temp.append(min(riga))
        min_dist = min(temp)
        if min_dist == distanza_infinita: # vuol dire non c è piu niente da assegnare
            break
        indice_riga_min= temp.index(min(temp))
        indice_colonna_min=indici_colonna_min[indice_riga_min]
        print(' min dist = ' , min_dist ,' indice riga e colonna = ', indice_riga_min , indice_colonna_min)
        assegnazione_rob_hum.append([freerobs[indice_colonna_min] , hums_liberi[indice_riga_min]])
        print(' assegnazione rob ' , freerobs[indice_colonna_min].rob_id , '  hum ' , hums_liberi[indice_riga_min][0].hum_id )
        #adesso levo riga e colonna selezionati
        matrice_distanze.pop(indice_riga_min)
        print(' matrice distanze dopo aver tolto riga = ' , matrice_distanze)
        for riga in matrice_distanze:
            riga.pop(indice_colonna_min)
        print(' matrice distanze dopo aver tolto colonne = ' , matrice_distanze)
        # levo i rib e hum assegnati dai risepttivi vettori
        freerobs.pop(indice_colonna_min)
        hums_liberi.pop(indice_riga_min)

    print(' matrice distanze dovrebbe essere vuota: matrice distanze = ' , matrice_distanze)
    #print(' assegnazione: (rob , hum)' , assegnazione_rob_hum[0][0].rob_id, assegnazione_rob_hum[0][1][0].hum_id , assegnazione_rob_hum[1][0].rob_id, assegnazione_rob_hum[1][1][0].hum_id ) #, assegnazione_rob_hum[2][0].rob_id, assegnazione_rob_hum[2][1].hum_id)
    return assegnazione_rob_hum

def ottimizza_globale_distanze(robs: List[MobileRobot], hums_to_serve: List[List[Human]] , missions: List[Mission] ):
        # ordino in base la numero di pattern all interno di una missione. ordinamento crescente.
        # partendo infatti nella funzione dall analizzare le missioni con meno pattern, ci dovrebbero essere meno persone coivolte
        # e c è piu possibilita che non si ripetano all interno di altre missioni. cosi posso assegnare piu missioni possibili

    hums_to_serve.sort(key=len)

    print(' controllo ordinamento')
    for hums in hums_to_serve:
        print('primo hum dell hums analizzato è hums_id = ' , hums[0].hum_id)
    print(' vorrei che hums to serve siano ordinati in base al numero crecente degli hum all interno di ogni hums')

    freerobs = free_robs(robs)
    hums_liberi = []
    missions_libere=[]
    for hums in hums_to_serve:
        if all_hums_free(hums):
            hums_liberi.append(hums)
            for hum in hums:
                hum.free = False
            missions_libere.append(missions[hums_to_serve.index(hums)])
    for hums in hums_to_serve:
        for hum in hums:
            hum.free = True
    #prendo i punti dei rob e degli umani
    punti_rob=[]
    for rob in freerobs:
        pos=rob.get_position()
        point_rob= Point(pos.x , pos.y)
        punti_rob.append(point_rob)
    punti_hum=[]
    for hums in hums_liberi:
        if missions_libere[hums_liberi.index(hums)].p[0] == Pattern.HUM_RECIPIENT or missions_libere[hums_liberi.index(hums)].p[0] == Pattern.HUM_INTER:
            # con human recipient ci sono 2 stage, non ha senso calcolare all inizio la distanza hum-rob perche quella è la stage 2
            # calcolo la distanza a cui deve arrivare prima di entrare in stage 2
            # poi la metto in point hum anche se non è un punto dell umano
            punti_hum.append(missions_libere[hums_liberi.index(hums)].dest[0])
            print(' sono qui')
            continue
        pos=hums[0].get_position()
        point_hum= Point(pos.x , pos.y)
        punti_hum.append(point_hum)
    # adesso creo la matrice
    matrice_distanze=[]
    for point_rob in punti_rob:
        distanza_hums_rob=[]
        for point_hum in punti_hum:
            distance= point_rob.distance_from(point_hum)
            distanza_hums_rob.append(distance)
        matrice_distanze.append(distanza_hums_rob)
    # righe ( = vettori) = distanza rob con i vari hums
    # colonne = distanza hum con i vari robs
    print(' matrice distanze = ', matrice_distanze)
    assegnazione_rob_hum=[]
    while len(matrice_distanze) != 0 and len(matrice_distanze[0]) != 0:
        print(' len matrice distanze = ', len(matrice_distanze))
        temp=[]
        indici_colonna_min=[]
        for riga in matrice_distanze:
            indice_colonna_min=riga.index(min(riga))
            indici_colonna_min.append(indice_colonna_min)
            temp.append(min(riga))
        min_dist = min(temp)
        indice_riga_min= temp.index(min(temp))
        indice_colonna_min=indici_colonna_min[indice_riga_min]
        print(' min dist = ' , min_dist ,' indice riga e colonna = ', indice_riga_min , indice_colonna_min)
        assegnazione_rob_hum.append([freerobs[indice_riga_min] , hums_liberi[indice_colonna_min]])
        print(' assegnazione rob ' , freerobs[indice_riga_min].rob_id , '  hum ' , hums_liberi[indice_colonna_min][0].hum_id )
        #adesso levo riga e colonna selezionati
        matrice_distanze.pop(indice_riga_min)
        print(' matrice distanze dopo aver tolto riga = ' , matrice_distanze)
        for riga in matrice_distanze:
            riga.pop(indice_colonna_min)
        print(' matrice distanze dopo aver tolto colonne = ' , matrice_distanze)
        # levo i rib e hum assegnati dai risepttivi vettori
        freerobs.pop(indice_riga_min)
        hums_liberi.pop(indice_colonna_min)

    print(' matrice distanze dovrebbe essere vuota: matrice distanze = ' , matrice_distanze)
    #print(' assegnazione: (rob , hum)' , assegnazione_rob_hum[0][0].rob_id, assegnazione_rob_hum[0][1][0].hum_id , assegnazione_rob_hum[1][0].rob_id, assegnazione_rob_hum[1][1][0].hum_id ) #, assegnazione_rob_hum[2][0].rob_id, assegnazione_rob_hum[2][1].hum_id)
    return assegnazione_rob_hum

def ordina_per_durata_mission_decrescente(missions : List[Mission]):	# passargli una copia del vettore

    new_list=[]
    lista_durate=[]
    for mis in missions:
        lista_durate.append(durata_mission(mis.p[:]))
    print(' lista durate = ' , lista_durate)
    while len(missions) != 0:
        for mis in missions:
            if num_piu_alto(lista_durate[missions.index(mis)] , lista_durate ):
                new_list.append(mis)
                lista_durate.pop(missions.index(mis))
                missions.remove(mis)
                break
    return new_list

#           FORSE INVECE CHE FARE DUE FILE.TXT ME NE BASTA 1 ?? FACCIO LA PROVA SEMPRE CON TEMPO INFINITO E MI
# OTTENGO LA CARICA BATTERIA CHE AL 95% PORTA IN FONDO LA MISSIONE
# TANTO IO LA DURATA LA CALCOLEREI SOLO PER POI OTTENERE CARICA MIN MISSION, A QUEL PPUNTO FACCIO UN PASSAGGIO IN MENO

# CAMBIARE SOLO DOVE RIUSO NEL CODICE LA DURATA_MISSION E METTERCI LA CHARGE, SE POI LEVO LA FUNZIONE DURATA_MISSION


def carica_minima_mission(mis : Mission):       # rifare il file time-battery

    durata=durata_mission(mis.p[:])     # da capire cosa passare alla funzione. gli passo mis.mission_id ??

    f = open('../scene_logs/time-battery.log', 'r+')
    lines = f.read().splitlines()
    i=0
    last_line=lines[i]
    while int(last_line.split(':')[1]) < durata:
        i+=1
        last_line=lines[i]
    return int(last_line.split(':')[0])

def durata_mission(id: int):
    f = open('../scene_logs/durata_mission{}'.format(id), 'r+') # devo ancora crearlo questo file, e magari lo metto in un altra cartella
    # estraggo la durata
    durata = 40
    return durata

def durata_mission2(patterns: List[Pattern]): # è quella vecchia	# passare copia         # leggere il file che mi viene dalla verifica formale fatta a monte
    if len(patterns)==0:
        return 0
    if patterns[0] == Pattern.HUM_LEADER:
        patterns.pop(0)
        t= 10 							        # cambiare dato. mettere come stima la media dei tempi dei test
        return t + durata_mission2(patterns)
    if patterns[0] == Pattern.HUM_FOLLOWER:
        patterns.pop(0)
        t= 15									# LETTURA FILE...... devo passare alla funzione anche la copia della lista degli umani
                                                # e delle destinazioni. ad ogni iterazione levo il primo elemento d entrambi,
                                                # anche sugli altri pattern ovviamente
                                                # nel caso di questo pattern infatti calcolo la distanza destinazione - umano
                                                # e poi guardo il file ( sara composto da durata:distanza e poi cerco la durata giusta )
        return t + durata_mission2(patterns)
    if patterns[0] == Pattern.HUM_RECIPIENT:
        patterns.pop(0)
        t1= 8	                                # cambiare dato. mettere come stima la media dei tempi dei test con cui rob arriva a dest
        t2 = 12                                  # prendere dato hum leader
        t = t1 +t2
        return t + durata_mission2(patterns)
    if patterns[0] == Pattern.HUM_INTER:
         patterns.pop(0)
         t1 = 11	                          # prendo il maggiore tra t10 e t11. t10 lo calcolo come hum follower. t11 = hum recipient stage 1
         t2 = 13                              # prendere dato hum leader
         t = t1 +t2
         return t + durata_mission2(patterns)

# posso essere preciso guardando alla ditanza hum - destinazione (hum leader , nuovo pattern stage = 1 , in entrambi entra in gioco
# la distanza e il frewill )
# su hum leader, hum recipient stage = 2 e nuovo pattern stage 2, la fine della mission dipende interamente dall uomo --> prove per fare una stima
# le parti influenzate dai rob non le posso calcolare bene (nuovo pattern stage = 1 , hum recipient stage =1 )
# perche io faccio una stima della durata prosprio per assegnare rob   --> prove per fare una stima
# in realta anche hum leader quando all inizio rob si deve avvicinare all umano, rientra in questa categoria, ma considero
# trascurabile questo tempo


# quindi :
# hum follower: leggo file, in base a dove deve andare l uomo cambia il tempo. non considero che hum si deve avvicinare prima al rob per farlo partire
# hum leader: la stima del tempo è sempre uguale per questo patter, è la media dei tempi delle prove effettuate in passato
# hum recipient: stage 1 stima non cambia ed è uguale alla media delle prove con cui rob raggiunge destinazione
# hum recipient: stage 2 prendo il dato di hum leader
# nuovo pattern: stage 1 lo tratto come hum follower, non considero il rob che arriva alla destinazione presupponendo sia piu corto
# nuovo pattern: stage 2 prendo dato human leader





def ordina_hums( humans: List[List[Human]], var: int):	# passare copia	# humans= lista di liste hums= lista hum = umano
    new_list=[]
    if var == 1:
        importanza_primi=[]
        for hums in humans:
            importanza_primi.append(hums[0].character.get_importance())
        while len(humans) != 0:
            new_list.append(humans[importanza_primi.index(max(importanza_primi))])
            humans.pop(importanza_primi.index(max(importanza_primi)))
            importanza_primi.remove(max(importanza_primi))
    if var == 2:
        lista_importanze=[]
        for hums in humans:
            lista_importanze.append(importanza_totale(hums[:]))
        print(' lista importanze', lista_importanze)
        while len(humans) != 0:
            for hums in humans:
                if num_piu_alto( lista_importanze[humans.index(hums)] , lista_importanze ):
                    new_list.append(hums)
                    lista_importanze.pop(humans.index(hums))
                    humans.remove(hums)
                    break
    return new_list

def num_piu_alto(num: int, array: List[int]):   # è un max(array), non c era bisogno
    for x in array:
        if x > num:
            return False
    return True

def importanza_totale(hums: List[Human]):
    if len(hums) == 0:
        return 0
    return hums[0].character.get_importance() + importanza_totale(hums[1:])




# QUESTO SARA POI L ORCH GENERALE

try:
    # START ROS NODES THAT ACQUIRE DATA FROM SENSORS
    time.sleep(5)
    print('START READING DATA HUMANS AND ROBOTS')
    start_reading_data(humans)
    start_reading_data_robs(robs)
    #time.sleep(5)  # in secondi            # con meno secondi a volte non faccio in tempo a leggere dati e mi da IndexError

    print('START MONITORING HUMANS SENSOR DATA LOGS')
    # START MONITORING HUMAN SENSOR DATA LOGS
    time.sleep(5)
    thread_h = Thread(target=follow_position, args=[humans])  # thread fa partire un processo che viene eseguito simultaneamente alle successive istruzioni del codice. cosi posso far partire piu processi insieme e monitorarli nello stesso tempo
    thread_h.start()
    time.sleep(5)
    thread_h_f = Thread(target=follow_fatigue, args=[humans])
    thread_h_f.start()

    print('START MONITORING ROBOTS SENSOR DATA LOGS')
    time.sleep(5)
    for rob in robs:
        time.sleep(2)
        thread_r = Thread(target=follow_position_rob, args=[rob])
        thread_r.start()
        thread_rb = Thread(target=rob.follow_charge)
        thread_rb.start()


    # START MISSION
    time.sleep(5)
    orch_list=[]
    orch_id=1
    threads=[]
    #se c è qualche rob sotto la soglia di batteria lo metto occupato e lo mando a recharge
    time.sleep(10)
    for rob in robs:
        if rob.get_charge() < rob.recharge_th :
            rob.free=False
            Thread(target=rob_charged, args=[rob]).start()

    # ORDINAMENTO


    # print(' prima dell ordinamento')
    # print('hums_to_serv ordine (id del primo hum ): ', hums_to_serve[0][0].hum_id , hums_to_serve[1][0].hum_id , hums_to_serve[2][0].hum_id)
    # print('missions ordine ( id mission): ',  total_missions[0].mission_id , total_missions[1].mission_id , total_missions[2].mission_id)
    # print(' x = ', x ,' var = ', var)


    if ottimizzazione == 1 :
        new_hums_to_serve = ordina_hums ( hums_to_serve[:] , ordinamento_hums )
        new_total_missions=[]
        for new_hums in new_hums_to_serve:
            for hums in hums_to_serve:
                if hums == new_hums:
                    new_total_missions.append(total_missions[hums_to_serve.index(hums)])
        hums_to_serve = new_hums_to_serve
        total_missions = new_total_missions

    if ottimizzazione == 2 or ottimizzazione == 4 or ottimizzazione == 5:
        new_total_missions = ordina_per_durata_mission_decrescente(total_missions[:])
        new_hums_to_serve=[]
        for new_mission in new_total_missions:
            for mis in total_missions:
                if mis == new_mission:
                    new_hums_to_serve.append(hums_to_serve[total_missions.index(mis)])
        hums_to_serve = new_hums_to_serve
        total_missions = new_total_missions
        if metodo_assegnazione_rob == 1:
            metodo_assegnazione_rob = 3
        if (ottimizzazione == 2 and ordine_crescente) or ottimizzazione == 4 or ottimizzazione == 5: # per 4 e 5 in ogni caso mi conviene invertirli
            total_missions = inverti_ordine_mis(total_missions[:])
            hums_to_serve = inverti_ordine_humss(hums_to_serve[:])

    if ottimizzazione == 3:
        assegnazione_rob = ottimizza_globale_distanze( robs[:] , hums_to_serve[:] , total_missions[:] )
        # è il caso piu appropriato
        metodo_assegnazione_rob = 1 #mi serve in sync

    if ottimizzazione == 4:
        # ho gia ordinato sopra in base alla durata decrescente. ora mi serve per durata crescente
        # total_missions = inverti_ordine_mis(total_missions[:])
        # hums_to_serve = inverti_ordine_humss(hums_to_serve[:])
        # queste due funzioni non le metto dentro perche poi quando adevo assegnare quelle successive è meglio partire analizando
        # quelle meno cariche cosi ho piu possibilita di assegazione

        # adesso sono ordinati con durata mission crescente. questo perche, per come funziona la funzione sotto, ho piu possibilita di assegnazione. ho piu possibilita di non mettere alcune distanze ad infinito
        assegnazione_rob = ottimizza_globale_batteria_dist( robs[:] , total_missions[:] , hums_to_serve[:])
        metodo_assegnazione_rob=3   # mi serve in sync

    if ottimizzazione == 5:
        metodo_assegnazione_rob=4   # deve essere dichiarato prima della funzione di ottimizzazione     # mi serve in sync
        # total_missions = inverti_ordine_mis(total_missions[:])
        # hums_to_serve = inverti_ordine_humss(hums_to_serve[:])
        assegnazione_rob = ottimizza_globale_batteria(robs[:] , total_missions[:] , hums_to_serve[:])



    # print(' dopo dell ordinamento')
    # print('hums_to_serv ordine (id del primo hum ): ', hums_to_serve[0][0].hum_id , hums_to_serve[1][0].hum_id , hums_to_serve[2][0].hum_id)
    # print('missions ordine ( id mission): ',  total_missions[0].mission_id , total_missions[1].mission_id , total_missions[2].mission_id)




    while len(total_missions)!=0:


        # ci puo essere qualche rob non assegnato che comunque ha bisogno di andare in reacharge
        for rob in robs:
            if rob.free == True and rob.get_charge() < rob.recharge_th :
                rob.free=False
                Thread(target=rob_charged, args=[rob]).start()

        # controllo orch per la sync
        while need_sync(orch_list):           # DA TESTARE !!!!!!!!!!!!!!!!!!!!!
            for orc in orch_list:	# se se ne liberano piu contemporaneamente, do la precedenza a quella piu importante per come è costruta l orch_list
                                 # nel caso ottimizzazione globale non è piu quello piu importante e neanche potrebbe essere quello che si libera prima
                                 # considero comunque le sync come eventi sporadici, quindi 2 contemporaneamente non dovrebbero capitare
                if orc.need_sync==True:
                    orch_need_sync=orc
                    break
            orch_need_sync.need_sync=False
            time.sleep(1) # cosi sono sicuro che nell orch.py la run mission vecchia è stata chiusa
            while len(free_robs(robs))==0:
                pass

            # la carica_minima_accettabile la calcolo sulla parte rimanente della mission
            missione_rimanente = Mission(orch_need_sync.mission.p[orch_need_sync.opchk.currH:] , orch_need_sync.mission.dest[orch_need_sync.opchk.currH:] , 10) # id casuale, questa mission mi serve solo per calcolare la carica minima accettabile sui pattern rimanenti
            carica_minima_accettabile = carica_minima_mission(missione_rimanente)
            rob_chosen=scegli_rob(robs,orch_need_sync.opchk.humans[orch_need_sync.opchk.currH] , metodo_assegnazione_rob , carica_minima_accettabile)
            if (orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_RECIPIENT or orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_INTER) and orch_need_sync.opchk.rec_stages==1 and (metodo_assegnazione_rob == 1 or metodo_assegnazione_rob == 3):
                if metodo_assegnazione_rob == 1:
                    rob_chosen = nearest_to_point(free_robs(robs), orch_need_sync.mission.dest[orch_need_sync.opchk.currH])
                if metodo_assegnazione_rob == 3:
                    rob_chosen = nearest_to_point(robs_abbastanza_carichi(free_robs(robs), carica_minima_accettabile) , orch_need_sync.mission.dest[orch_need_sync.opchk.currH])
            # qualche rob libero c è, se nessuno rientra nel caso migliore io lo stesso gli devo assegnare qualche rob
            if rob_chosen == None and (metodo_assegnazione_rob == 3 or metodo_assegnazione_rob == 4 ):
                var_rob_t=2
                rob_chosen=scegli_rob(robs,orch_need_sync.opchk.humans[orch_need_sync.opchk.currH] , var_rob_t , carica_minima_accettabile)


            print(' rob id for sync =  ', rob_chosen.rob_id)
            rob_chosen.free=False
            if orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_FOLLOWER:
                Thread(target=rob_go_to_hum_and_sync, args=[orch_need_sync, rob_chosen, orch_id]).start()
            if orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_LEADER or orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_RECIPIENT or orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_INTER:
                if (orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_RECIPIENT or orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_INTER) and orch_need_sync.opchk.rec_stages==2:
                    # faccio arrivare il vecchio rob dall hum, quello nuuovo dall hum, e poi dall orchestator.py quello vecchio in recharge
                    Thread(target=sync_in_rec_stages2, args=[orch_need_sync,rob_chosen,orch_id]).start()
                else:
                    Thread(target=sync, args=[orch_need_sync, rob_chosen, orch_id]).start()
            orch_id+=1



        # prima di andare sul for ricontrollo se qualche mission si è chiusa
        for mission in total_missions:
            if mission.get_scs():
                hums_to_serve.pop(total_missions.index(mission))
                total_missions.remove(mission)

# fare prove con synch all inizio in mezzo in fondo e lo stesso con piu sync
# anche prova che assegno missione, poi sinc, e poi assegno una nuova missione ( tipo se ci fosero 2 rob e 3 missioni ma all inizio subito una ha bisogno di sync

# !!!!!!!!!!!!!!!!!!!!!!!!!

        if (ottimizzazione == 3 or ottimizzazione == 4 or ottimizzazione == 5) and len(assegnazione_rob)!=0:
            while len(assegnazione_rob) != 0:
                print(' dentro al nuovo while')
                assigned_hums = assegnazione_rob[0][1]
                for hum in assigned_hums:
                    hum.free = False
                rob_chosen = assegnazione_rob[0][0]
                rob_chosen.free=False
                print(' prima di assigned mission')
                assigned_mission = total_missions[hums_to_serve.index(assigned_hums)]
                print(' dopo di assigned mission. mssion id = ' , assigned_mission.mission_id , ' primo hum id = ' , assigned_hums[0].hum_id)
                opchk = OpChk(0.5, 0.0, rob_chosen, assigned_hums, assigned_mission)
                orch = Orchestrator(opchk, orch_id)
                orch_id+=1
                orch_list.append(orch)
                threads.append(Thread(target=orch.run_mission))
                threads[-1].start()
                assegnazione_rob.pop(0)
            continue        # cosi ritorno all inizio del while e non faccio le condizioni dopo
# ci puo essere qualche rob non assegnato che comunque ha bisogno di andare in reacharge
        for rob in robs:
            if rob.free == True and rob.get_charge() < rob.recharge_th :
                rob.free=False
                Thread(target=rob_charged, args=[rob]).start()

        for hums in hums_to_serve:          # hums_to_serve è lista di liste. hums è lista
            if all_hums_free(hums)==False:  # per ogni insieme di umani assegnati ad ogni missione, controllo che questi siano liberi prima di iniziare la missione
                continue                    # il pass non ti fa fare niente. il continue ti fa andare alla prossima iterazione del for

            if len(free_robs(robs)) == 0:
                break

            # se ottimizzazione è qualsiasi altro numero anche != da 1 e 2 vuol dire non ottimizzo niente ma qualcosa devo assegnare
            if ottimizzazione != 3 or ottimizzazione !=4 or ottimizzazione != 5:   # ci puo non essere assegnazione se: non ci sono rob o con metodo_ass_rob = 3
                assigned_mission = total_missions[hums_to_serve.index(hums)] # trovare l indice giusto della missione
                carica_minima_accettabile = carica_minima_mission(assigned_mission)
                rob_chosen=scegli_rob(robs , hums[0] , metodo_assegnazione_rob , carica_minima_accettabile)
                if (assigned_mission.p[0] == Pattern.HUM_RECIPIENT or assigned_mission.p[0] == Pattern.HUM_INTER) and (metodo_assegnazione_rob == 1 or metodo_assegnazione_rob == 3):
                    if metodo_assegnazione_rob == 1:
                        rob_chosen = nearest_to_point(free_robs(robs), assigned_mission.dest[0])
                    if metodo_assegnazione_rob == 3:
                        rob_chosen = nearest_to_point(robs_abbastanza_carichi(free_robs(robs), carica_minima_accettabile) , assigned_mission.dest[0])

            if ottimizzazione == 3:    # ci puo non essere assegnazione se: non ci sono rob.
                # cè un solo rob libero perche si è appena liberato, senno sarebbe stato assegnato da prima.
                # per sicurezza pero stampo
                print(' len(freerobs(robs)) = ', len(free_robs(robs)) , ' ( . Dovrebbe venire sempre = 1')
                rob=free_robs(robs)[0]
                hums_liberi=[]
                missions_libere=[]
                for hums in hums_to_serve:
                    if all_hums_free(hums):
                        hums_liberi.append(hums)
                        missions_libere.append(total_missions[hums_to_serve.index(hums)])
                pos=rob.get_position()
                point_rob= Point(pos.x , pos.y)
                distanze=[]
                for hums in hums_liberi:
                    if missions_libere[hums_liberi.index(hums)].p[0] == Pattern.HUM_RECIPIENT or missions_libere[hums_liberi.index(hums)].p[0] == Pattern.HUM_INTER:
                         # con human recipient ci sono 2 stage, non ha senso calcolare all inizio la distanza hum-rob perche quella è la stage 2
                        # calcolo la distanza a cui deve arrivare prima di entrare in stage 2
                        # poi la metto in point hum anche se non è un punto dell umano
                        point_hum=missions_libere[hums_liberi.index(hums)].dest[0]
                        distanze.append(point_rob.distance_from(point_hum))
                        continue
                    pos_hum=hums[0].get_position()
                    point_hum= Point(pos_hum.x , pos_hum.y)
                    distanze.append(point_rob.distance_from(point_hum))
                hums_scelti=hums_liberi[distanze.index(min(distanze))]

                assigned_mission=total_missions[hums_to_serve.index(hums_scelti)]
                rob_chosen=rob
                hums=hums_scelti

            if ottimizzazione == 4:      # ci puo non essere assegnazione se: non ci sono rob o se tutte le cariche_min sono > di carica_rob
                # ci possono essere piu mission  e piu rob, coppie scartate dall assegnazione iniziale
                rob=rob_piu_carico_tra_i_liberi(robs)
                print('ho preso il rob ' , rob.rob_id)
                missions_libere=[]
                hums_liberi=[]
                for hums in hums_to_serve:
                    if all_hums_free(hums):
                        hums_liberi.append(hums)
                        missions_libere.append(total_missions[hums_to_serve.index(hums)])
                carica_rob=rob.get_charge()
                print(' carica rob = ' , carica_rob)
                carica_min_tutte= 100
                # con il for se poi levo degli elementi dal vettore, mi sballa gli indici per i cicli successivi del for
                # per questo ho bisogno di altri vettori ( hum accettabili e mission accettabili
                missions_accettabili=[]
                hums_accettabili=[]
                for mis in missions_libere:
                    carica_min= carica_minima_mission(mis)
                    print('carica min mission ', mis.mission_id , ' = ' , carica_min)
                    if carica_min >= carica_rob:
                        if carica_min<carica_min_tutte:
                            carica_min_tutte=carica_min
                            assigned_mission_2=mis
                            hums2=hums_liberi[missions_libere.index(assigned_mission_2)]
                        # hums_liberi.pop(missions_libere.index(mis))           # li ho commentati perche levando elementi da quei vettori poi il ciclo for non funziona piu con gli indici
                        # missions_libere.remove(mis)
                    if carica_min < carica_rob:
                        missions_accettabili.append(mis)
                        hums_accettabili.append(hums_liberi[missions_libere.index(mis)])
                if len(missions_accettabili) != 0:
                    print(' entro quiiii')
                    # adesso come prima nel caso ottimizzazione == 3
                    pos=rob.get_position()
                    point_rob= Point(pos.x , pos.y)
                    distanze=[]
                    for hums in hums_accettabili:
                        if missions_accettabili[hums_accettabili.index(hums)].p[0] == Pattern.HUM_RECIPIENT or missions_accettabili[hums_accettabili.index(hums)].p[0] == Pattern.HUM_INTER:
                            # con human recipient ci sono 2 stage, non ha senso calcolare all inizio la distanza hum-rob perche quella è la stage 2
                            # calcolo la distanza a cui deve arrivare prima di entrare in stage 2
                            # poi la metto in point hum anche se non è un punto dell umano
                            point_hum=missions_accettabili[hums_accettabili.index(hums)].dest[0]
                            distanze.append(point_rob.distance_from(point_hum))
                            continue
                        pos_hum=hums[0].get_position()
                        point_hum= Point(pos_hum.x , pos_hum.y)
                        distanze.append(point_rob.distance_from(point_hum))
                    print(' stampo vettore')
                    print(distanze)
                    min_dist=min(distanze)
                    print(min_dist)
                    hums_scelti=hums_accettabili[distanze.index(min_dist)]
                    assigned_mission=total_missions[hums_to_serve.index(hums_scelti)]
                    rob_chosen=rob
                    hums=hums_scelti
                else:           # prendo la mission che ha carica_min minore, cosi ho piu possibilita di arrivare al scs
                    assigned_mission=assigned_mission_2
                    rob_chosen=rob
                    hums=hums2

            if ottimizzazione == 5: # ci puo non essere assegnazione se non ci sono rob liberi o tutte cariche_min > carica_rob
                 # ci possono essere piu rob. le mission sono gia ordinate in ordine crescente di durata
                 # posso arrivare a questo punto se si libera una missione oppure se dopo l assegnazione iniziale ci sono delle missioni
                 # che durano troppo per i rob che ci sono oppure un mix delle due se tipo si liberano gli umani di una missione che pero dura troppo.
                 #
                 # ogni volta che entro qui dentro ( ci arrivo solo se c è almeno un rob libero) prendo il rob piu carico e a quello
                 # se riesco assegno la mission che dura di piu tra quelle con durata accettabile, altrimenti assegno quella che dura
                 # di meno e intanto la faccio partire
                rob=rob_piu_carico_tra_i_liberi(robs)
                missions_libere=[]
                print('len ' , len(missions_libere))
                hums_liberi=[]
                for hums in hums_to_serve:
                    if all_hums_free(hums):
                        hums_liberi.append(hums)
                        missions_libere.append(total_missions[hums_to_serve.index(hums)])
                        print('len ' , len(missions_libere))
                carica_rob=rob.get_charge()
                cariche_accettabili=[]
                carica_min_non_accettata=100
                missions_accettabili=[]
                hums_accettabili=[]
                for mis in missions_libere:
                    carica_min= carica_minima_mission(mis)
                    print(' carica min , carica rob = ' , carica_min , carica_rob)
                    if carica_min >= carica_rob:
                        if carica_min < carica_min_non_accettata:
                            carica_min_non_accettata=carica_min
                            assigned_mission_2=mis
                            hums2=hums_liberi[missions_libere.index(assigned_mission_2)]
                    if carica_min < carica_rob:
                        missions_accettabili.append(mis)
                        hums_accettabili.append(hums_liberi[missions_libere.index(mis)])
                        cariche_accettabili.append(carica_min)
                if len(missions_accettabili) != 0:
                    print('len missions_libere = ', len(missions_accettabili) , 'len cariche accetta = ', len(cariche_accettabili))
                    assigned_mission=missions_accettabili[cariche_accettabili.index(max(cariche_accettabili))]
                    rob_chosen=rob
                    hums=hums_accettabili[missions_accettabili.index(assigned_mission)]
                else:           # prendo la mission che ha carica_min minore, cosi ho piu possibilita di arrivare al scs
                    assigned_mission=assigned_mission_2
                    rob_chosen=rob
                    hums=hums2

            if rob_chosen == None and (metodo_assegnazione_rob == 3):
                var_rob_t=2
                rob_chosen=scegli_rob(robs , hums[0] , var_rob_t , carica_minima_accettabile)

            if rob_chosen==None:
                print(' QUI NON CI DOVREBBE MAI ENTRARE')
                break                      # voglio uscire dal ciclo for.
                # nel caso non lo facessi, ci possono essere piu gruppi di persone libere. quando sto analizzando uno meno importante
                # di altri e in quel momento mi si libera il rob, allora il rob mi viene assegnato un gruppo meno importante.
                # invece se ogni volta esco, quando mi si libera il rob, per forza viene assegnato al gruppo piu importante libero in quel momento
            #assigned_mission = total_missions[hums_to_serve.index(hums)] # trovare l indice giusto della missione levo e metto sopra
            opchk = OpChk(0.5, 0.0, rob_chosen, hums, assigned_mission)
            orch = Orchestrator(opchk, orch_id)
            orch_id+=1
            orch_list.append(orch)
            threads.append(Thread(target=orch.run_mission))
            threads[-1].start()
            for hum in hums:		# metto umani e rob come occupati
                    hum.free=False
            rob_chosen.free=False
            # ho messo nell orch. che una volta che la missione è andata in scs o in fail
            # sia umani coinvolti che rob scelto diventano free

        # missioni e rispetivi hums hanno lo stesso indice.
        # rimuovo missione e hums dai rispettivi vettori.
        # i thread che monitorano i dati degli umani però non li chiudo piu, perche poi possono servire su altre missions
        for mission in total_missions:
            if mission.get_scs():
                hums_to_serve.pop(total_missions.index(mission))
                total_missions.remove(mission)


    # when mission is over (either with success or failure),
    # shut everything down
    for hum in humans:
        hum.set_sim_running(0)
    for rob in robs:
        rob.set_sim_running(0)
        # se stanno andando in carica o sono in carica, lo sono in thred a parte dal main. con questa variabile faccio
        # terminare i thread nelle rispettive funzioni
        rob.execution_finished=True
    vrep.stop_sim(vrep_sim)

    time.sleep(7)
    print('Execution finished.')
    quit()

except (KeyboardInterrupt, SystemExit):
    for rob in robs:
        rob.set_sim_running(0)
    for hum in humans:
            hum.set_sim_running(0)
    vrep.stop_sim(vrep_sim)
    quit()



# 				COSE DA FARE:
#
#
#		capire perche non mi si stoppa human follower e recipient ( in rec_stages=2) se non è l ultimo pattern della mission
#
#
#
#	 nuovo pattern : anche questo, come recipient, lo faccio a due fasi. scorro orch.py e aggiungo i vari pezzi in base
#	al pattern che è piu simile in quella fase e prendo spunto. all ultimo faccio i  vari test
#
#	se umano si vuole fermare ( ultimo pattern) o vuole stoppare la misione ( leader e ultimo pattern)
#	 ho capito come fare su lua e su orch.py come leggerli. fare anche sul nuovo pattern. su questo
#	aggiungerci che puo anche andare in pausa e poi riprendere.
#	ora che ci penso se l umano vuole andare in pausa al rob non importa nulla perche rimene fermo. l hum puo fare
#	quello che vuole ( lavorare o pausa) e il rob sta li. al rob importa solo quando hum dice che ha finito






#	fare i vari test, con piu sincronizzazioni,... ( guarda quad e sopra) 	e se hum si affatica ( che ancora non l ho mai fatto)
# per i test sulle sync: annotarsi i casi che testo e il risultato perche ci possono essere molte combinazioni, cosi poi mi ricordo cosa ho fatto
#
#
#
#  CHE FACCIO CON IL NUOVO PATTERN ???????    ( NEL CASO MODIFICARE LUA E ORCH.PY PER AGGIUNGERE LA PAUSA
#
#  LO FACCIO IL FILE A PARTE DELL ORCH GENERALE ?????
#
#
#
