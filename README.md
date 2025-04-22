I dette mini-prosjektet har vi tatt utgangspunkt i labøving 1, 2 og 3 for å integrere og drifte en kube i den virkelige verden med å programmere den gjennom ROS2 som står for komunikasjonen med robotikken i kuben. Vi har fire pakker som inneholder hele prosjektet og er strukturert som dette:
- qube_description: Inneholder den geometriske beskrivelsen av Quben
- qube_driver: Inneholder kommunikasjonsgrensesnittet med den fysiske Quben
- qube_bringup: Inneholder launch- og konfigurasjonsfiler og dokumentasjon.
- qube_controller: Innholder en PID-kontroller som regulerer roboten.

Beskrivelse av hvordan systemet er satt opp
- I oppgave 1 beskriver vi roboten, eller "Qube" (kube) som vi refere den til, i en Xacro/URDF-fil. Filen definnerer leddene til roboten, samt samt linker og relasjoner til verden. Denne beskrivelsen tar også for seg ROS2 Control-konfigurasjon som gjør det mulig å anvende kontroller for å styre roboten via simulering eller maskinvaregensesnitt.

- I oppgave 2 knytter vi robotbeskrivelsen til kontrollsystemet med å inkludere ros2_control-controller manager, samt nødvendige hardware interface-konfigurasjoner i Xacro-filen. Dette lar det muliggjøres å starte en kontroll-node som kommuniserer med den fysiske hardwaren eller simulert modell basert på parameteret simulation.

- I Oppgave 3 settes det opp en "publiser" for robot tilstand som publiserer transformasjonen mellom robotens linker utfra sensor- og kontrollverdier. Noden lar oss lese "robot_description"-parametet fra parameter-serveren, og er avgjørende for at andre noder, som RViz skal kunne visualisere robotens nåværende tilstand.

- Oppgave 4 tar vi for oss hvordan dataen flyter mellom komponentene i systmet via ROS2 topics. Kontrolleren publiserer(publish) og abonnerer(subscribe) på topics som tar for segbåde komandoer og inngående måledata. det blir etablert topics som posisjon og hastighet som anvendes av "controll manager" og "robot_state_publisher.

- Til slutt åpnes RViz for å visualisere robotmodellen og sanntidsdataen fra transformasjonen og topics. Dette lar oss følge bevegelsene og posisjonen til roboten, samt simulerte og/eller fysiske tilstander.
