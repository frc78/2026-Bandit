# Indexer states

## Spindexer

| State              | Spindexer | Feeder  |
|--------------------|-----------|---------|
| Still              | Off       | Off     |
| Waiting for Turret | Forward   | Off     |
| Indexing           | Forward   | Forward |
| Auto Unjamming     | Reverse   | Reverse |
| Manual Unjamming   | Reverse   | Reverse |

```mermaid
stateDiagram-v2
    still
    waiting_for_turret
    state turret_ready <<choice>>
    indexing
    auto_unjamming
    manual_unjamming
    still --> waiting_for_turret: driver shoot button
    still --> manual_unjamming: driver unjam button
    indexing --> still: driver off shooter button
    indexing --> manual_unjamming: driver unjam button
    indexing --> waiting_for_turret: unaimed
    indexing --> auto_unjamming: detect jam
    manual_unjamming --> still: driver off unjam button
    manual_unjamming --> waiting_for_turret: driver off unjam button & driver shoot button
    auto_unjamming --> waiting_for_turret: sequence finished
    auto_unjamming --> still: drive cease fire
    waiting_for_turret --> turret_ready
    turret_ready --> Still: !shoot button
    turret_ready --> indexing: shoot button
```