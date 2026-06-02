Climbing States

Stowed

```mermaid
stateDiagram-v2
    * --> Unclimbed
    Climbed -->  Unclimbed: Deploy Climb
    Unclimbed --> Climbed: Retract Climb 
```
