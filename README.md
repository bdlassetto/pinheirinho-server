# Pinheirinho2 Race Server

Servidor autoritativo de arrancada (drag racing) do app **Pinheirinho2** para
Assetto Corsa — Brazilian Drag League (BDL).

- WebSocket (frames de texto JSON), porta via env `SERVER_PORT`/`PIN2_PORT` (padrão 8765)
- FSM da corrida a 120Hz: staging, árvore de largada, queima, RT, parciais (60ft/100m/201m)
- Papéis: 2 pilotos (left/right) + N espectadores
- Clock sync estilo NTP (`SYNC`/`SYNC_ACK`) para árvore sincronizada ao milissegundo
- Tokens: `PIN2_ADMIN_TOKEN` (reset do diretor, fail-closed) e `PIN2_TOKEN` (acesso, opcional)

## Rodar

```bash
pip install websockets
python server/server.py
```

Este repositório contém apenas o lado servidor. O app cliente (dentro do AC) é distribuído separadamente.
