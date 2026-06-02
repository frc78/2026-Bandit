```mermaid
stateDiagram-v2
    Retracted
    Deployed
    Intaking
    Jostle
    Outtaking
    Retracted --> Deployed: Deploy button pressed
    Deployed --> Intaking: Intake button pressed
    Deployed --> Outtaking: Outtake button pressed
    Deployed --> Jostle: Jostle button pressed
    Deployed --> Retracted: Deploy button pressed
    Intaking --> Retracted: Deploy button pressed
    Intaking --> Deployed: Intake button released
    Intaking --> Jostle: Jostle button pressed
    Jostle --> Deployed: Jostle button released
```