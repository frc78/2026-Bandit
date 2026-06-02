

Drive State
```mermaid
stateDiagram-v2
Field_oriented --> Robot_oriented
Robot_oriented --> Field_oriented
x_mode --> Field_oriented
Field_oriented --> x_mode
x_mode --> Robot_oriented
Robot_oriented --> x_mode
Pathplan --> x_mode
x_mode --> Pathplan
Robot_oriented --> Pathplan
Pathplan --> Robot_oriented
Field_oriented --> Pathplan
Pathplan --> Field_oriented
Aiming_feed --> Field_oriented
Field_oriented --> Aiming_feed
Pathplan --> Aiming_feed
Aiming_feed --> Pathplan
x_mode --> Aiming_feed
Aiming_feed --> x_mode
Robot_oriented --> Aiming_feed
Aiming_feed --> Robot_oriented
Aiming_Score --> Field_oriented
Field_oriented --> Aiming_Score
Aiming_Score --> Robot_oriented
Robot_oriented --> Aiming_Score
x_mode --> Aiming_Score
Aiming_Score --> x_mode
Aiming_Score --> Pathplan
Pathplan --> Aiming_Score
Aiming_Score --> Aiming_feed
Aiming_feed --> Aiming_Score
```

