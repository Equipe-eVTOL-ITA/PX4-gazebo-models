# wanda

O x500 em **escala 0,45**, com LIDAR 2D. Feito para a fase 4 da CBR 2026.

**GERADO** por `tools/gerar_wanda.py`. Não edite o `model.sdf` à mão — regere:

```bash
python3 tools/gerar_wanda.py
```

## Por que ele existe

O x500 tem **0,772 m** de envergadura e as janelas do labirinto da fase 4 têm
**0,60 m**. Faltam 8,6 cm de cada lado: não é apertado, é impossível.

| | x500 | wanda |
|---|---|---|
| envergadura | 0,772 m | **0,347 m** |
| massa | 2,06 kg | 0,50 kg |
| rotação de pairar | 77% do máx. | 77% do máx. |

## As leis de escala, e a que não se aplica

Comprimentos ×0,45. Inércia × (massa nova/velha) × 0,45². `motorConstant`
ajustado para o drone pairar na **mesma fração** da rotação máxima que o x500 —
é o que faz o controlador do PX4 encontrar um ponto de operação parecido.

**A massa não segue o cubo.** Densidade constante daria 0,19 kg para um drone de
35 cm, o que é inviável: bateria, controladora e fiação não encolhem assim. Um
quadricóptero real de 35 cm com hélices de 6" pesa perto de 0,5 kg, e é esse o
valor. Consequência: o `wanda` é proporcionalmente **mais pesado** que o x500,
como todo drone pequeno de verdade é.

## O que este modelo não resolve

**Os ganhos de taxa do PX4.** Um drone quatro vezes mais leve e duas vezes menor
tem dinâmica de atitude muito mais rápida, e os ganhos do airframe `4001` — que
é do x500 — vão ficar altos.

Espere oscilação de atitude no primeiro voo. O ajuste é baixar
`MC_ROLLRATE_P` e `MC_PITCHRATE_P`, e provavelmente `MC_ROLLRATE_D` /
`MC_PITCHRATE_D` junto. Isso não dá para derivar de escala: mede-se voando.

## Verificado

- Spawna no `gz sim` e publica IMU, barômetro, magnetômetro, NavSat e o `/scan`.
- Offset do LIDAR **medido** em 0,053 m à frente do centro — que é 0,121 × 0,45,
  consistente com a escala. Esse número entra em `lidar_offset_frente` no
  `config/flight.yaml` da fase 4.
- `gz sdf -k` **não** serve para validar este modelo: o `x500_lidar_2d`, que
  funciona, também reprova nele. Includes mesclados confundem o validador.
