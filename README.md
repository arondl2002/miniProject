I dette mini-prosjektet har vi tatt utgangspunkt i labøving 1, 2 og 3 for å integrere og drifte en kube i den virkelige verden med å programmere den gjennom ROS2 som står for komunikasjonen med robotikken i kuben. Vi har fire pakker som inneholder hele prosjektet og er strukturert som dette:
- qube_description: Inneholder den geometriske beskrivelsen av Quben
- qube_driver: Inneholder kommunikasjonsgrensesnittet med den fysiske Quben
- qube_bringup: Inneholder launch- og konfigurasjonsfiler og dokumentasjon.
- qube_controller: Innholder en PID-kontroller som regulerer roboten.

Beskrivelse av hvordan systemet er satt opp
- I oppgave 1 en robot description, som beskriver quben, i en Xacro/URDF-fil. Filen definerer leddene til roboten, samt linker og relasjoner (joints) til verden. 

- Oppgave 2 inneholder en qube driver, som vil si kode som tar seg av hardware interfacing.

- I Oppgave 3 settes det opp en launch-fil som har ansvaret for å starte alle andre noder som skal kjøres i systemet. Den har en URDF-fil som binder sammen qube.macro.xacro fra oppgave 1, og qube_driver.ros2_control.xacro fra oppgave 2. URDF-filen har også argumenter som tillater å endre klokkehastigheten til kommunikasjonen (baud rate), enheten som driveren leter på (device) og om man skal simulere eller bruke en ekte qube med digital tvilling.

- Oppgave 4 inneholder en PID-kontroller som styres basert på en måleverdi, mottatt fra /joint_states. Den publiserer hastighetspådraget til /velocity_controller/commands. For å stille PID-verdier i etterkant kan man sette parameter verdier.
```bash
ros2 param set /qube_controller_node *parameter navn* *float verdi*
```
Mulige parameternavn er 'kp', for P verdi, 'ki', for I verdi, 'kd', for D verdi og 'setpoint' for setpunktverdi

