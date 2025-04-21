I dette mini-prosjektet har vi tatt utgangspunkt i labøving 1, 2 og 3 for å integrere og drifte en kube i den virkelige verden med å programmere den gjennom ROS2 som står for komunikasjonen med robotikken i kuben. Vi har fire pakker som inneholder hele prosjektet og er strukturert som dette:
- qube_description: Inneholder den geometriske beskrivelsen av Quben
- qube_driver: Inneholder kommunikasjonsgrensesnittet med den fysiske Quben
- qube_bringup: Inneholder launch- og konfigurasjonsfiler og dokumentasjon.
- qube_controller: Innholder en PID-kontroller som regulerer roboten.
