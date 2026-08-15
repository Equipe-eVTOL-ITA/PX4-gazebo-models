#!/usr/bin/env python3
"""
Gera o modelo `wanda` a partir do x500, em escala menor.

    python3 tools/gerar_wanda.py

POR QUE UM SCRIPT, E NAO UM ARQUIVO EDITADO A MAO

Escalar um multirrotor nao e encolher a geometria. Massa, inercia e as
constantes do motor seguem leis diferentes, e errar qualquer uma delas da um
modelo que carrega no Gazebo, spawna, e nao voa -- ou voa e nao se parece com
nada. Deixando a derivacao em codigo, as leis ficam escritas e conferiveis, e
regerar depois de mexer numa premissa custa um comando.

AS LEIS DE ESCALA USADAS AQUI

  comprimentos      x k          poses, caixas, escalas de malha
  massa             ESCOLHIDA    ver abaixo -- NAO e k^3
  inercia           x (m'/m) k^2  a forma nao muda, so tamanho e massa
  motorConstant     x (m'/m)     mantem a rotacao de pairar identica
  momentConstant    x k          e uma razao torque/empuxo, com unidade de
                                 comprimento

A MASSA NAO ESCALA COM O CUBO.

Densidade constante daria 2.064 kg x 0.45^3 = 0.19 kg para um drone de 35 cm --
inviavel. Bateria, controladora e fiacao nao encolhem com o cubo do tamanho. Um
quadricoptero real de 35 cm com helices de 6" pesa perto de 0.5 kg, e e esse o
valor usado. Consequencia: `wanda` e proporcionalmente MAIS PESADO que o x500,
como todo drone pequeno de verdade e.

`motorConstant` e ajustado para que a rotacao de pairar caia na MESMA fracao da
maxima que a do x500 (cerca de 77%). E o que faz o controlador do PX4 encontrar
um ponto de operacao parecido, em vez de saturar ou flutuar perto de zero.

O QUE ESTE SCRIPT NAO RESOLVE

Os ganhos de taxa do PX4. Um drone quatro vezes mais leve e duas vezes menor tem
dinamica de atitude muito mais rapida, e os ganhos do airframe 4001 -- feito
para o x500 -- vao ficar altos. Espere oscilacao de atitude no primeiro voo e
ajuste MC_ROLLRATE_P / MC_PITCHRATE_P para baixo. Isto esta anotado no README
do modelo tambem.
"""

from __future__ import annotations

import copy
import math
import pathlib
import xml.etree.ElementTree as ET

RAIZ = pathlib.Path(__file__).resolve().parent.parent

# ── As premissas, num lugar so ──────────────────────────────────────────────

K = 0.45                 # fator de escala linear
MASSA_ALVO = 0.50        # kg, o drone inteiro (ver o cabecalho)
NOME = "wanda"

# Do x500, lidos do model.sdf dele.
MASSA_X500 = 2.0 + 4 * 0.016076923076923075
MOTOR_CONST_X500 = 8.54858e-06
MOMENT_CONST_X500 = 0.016
MAX_ROT_X500 = 1000.0

RAZAO_MASSA = MASSA_ALVO / MASSA_X500


def escalar_vetor(texto: str, n: int, fator: float) -> str:
    """Multiplica os `n` primeiros numeros de um texto separado por espacos."""
    v = texto.split()
    for i in range(min(n, len(v))):
        v[i] = f"{float(v[i]) * fator:.6g}"
    return " ".join(v)


def escalar_arvore(no: ET.Element) -> None:
    """Aplica as leis de escala, recursivamente."""
    for e in no.iter():
        # Poses: as TRES primeiras componentes sao translacao; as tres ultimas
        # sao angulos e NAO escalam. Escalar a rotacao junto e o erro classico
        # -- ele deixa o drone montado torto, e nada acusa.
        if e.tag == "pose" and e.text:
            e.text = escalar_vetor(e.text, 3, K)

        elif e.tag in ("size", "scale") and e.text:
            e.text = escalar_vetor(e.text, 3, K)

        elif e.tag in ("radius", "length") and e.text:
            e.text = f"{float(e.text) * K:.6g}"

        elif e.tag == "mass" and e.text:
            e.text = f"{float(e.text) * RAZAO_MASSA:.6g}"

        elif e.tag in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz") and e.text:
            # A forma nao muda: I = m * L^2 * (fator de forma).
            e.text = f"{float(e.text) * RAZAO_MASSA * K * K:.6g}"


def motor(indice: int, junta: str, elo: str, sentido: str) -> ET.Element:
    p = ET.Element("plugin", {
        "filename": "gz-sim-multicopter-motor-model-system",
        "name": "gz::sim::systems::MulticopterMotorModel"})
    campos = {
        "jointName": junta,
        "linkName": elo,
        "turningDirection": sentido,
        "timeConstantUp": "0.0125",
        "timeConstantDown": "0.025",
        "maxRotVelocity": f"{MAX_ROT_X500:g}",
        "motorConstant": f"{MOTOR_CONST_X500 * RAZAO_MASSA:.6g}",
        "momentConstant": f"{MOMENT_CONST_X500 * K:.6g}",
        "commandSubTopic": "command/motor_speed",
        "motorNumber": str(indice),
        "rotorDragCoefficient": f"{8.06428e-05 * RAZAO_MASSA:.6g}",
        "rollingMomentCoefficient": "1e-06",
        "rotorVelocitySlowdownSim": "10",
        "motorType": "velocity",
    }
    for k, v in campos.items():
        ET.SubElement(p, k).text = v
    return p


def main() -> int:
    origem = RAIZ / "models" / "x500_base" / "model.sdf"
    arv = ET.parse(origem)
    modelo = arv.getroot().find("model")
    modelo.set("name", NOME)
    escalar_arvore(modelo)

    # As malhas continuam sendo as do x500 -- so o `<scale>` muda, e ele ja foi
    # multiplicado acima. Referenciar o diretorio do x500 evita copiar 13
    # arquivos binarios para dentro deste modelo.

    # Os quatro motores, com as constantes reajustadas.
    for i, (junta, elo, sentido) in enumerate([
            ("rotor_0_joint", "rotor_0", "ccw"),
            ("rotor_1_joint", "rotor_1", "ccw"),
            ("rotor_2_joint", "rotor_2", "cw"),
            ("rotor_3_joint", "rotor_3", "cw")]):
        modelo.append(motor(i, junta, elo, sentido))

    # O LIDAR 2D, no topo. A altura tambem escala: num drone menor ele fica mais
    # baixo, e e dali que o scan sai.
    inc = ET.SubElement(modelo, "include")
    ET.SubElement(inc, "uri").text = "model://lidar_2d_v2"
    ET.SubElement(inc, "pose").text = f"{0.12 * K:.6g} 0 {0.26 * K:.6g} 0 0 0"
    inc.set("merge", "true")

    junta = ET.SubElement(modelo, "joint", {"name": "LidarJoint", "type": "fixed"})
    ET.SubElement(junta, "parent").text = "base_link"
    ET.SubElement(junta, "child").text = "link"
    ET.SubElement(junta, "pose", {"relative_to": "base_link"}).text = \
        f"{-0.1 * K:.6g} 0 {0.26 * K:.6g} 0 0 0"

    destino = RAIZ / "models" / NOME
    destino.mkdir(parents=True, exist_ok=True)
    ET.indent(arv, space="  ")
    arv.write(destino / "model.sdf", encoding="utf-8", xml_declaration=True)

    cfg = destino / "model.config"
    cfg.write_text(f"""<?xml version="1.0"?>
<model>
  <name>{NOME}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    O x500 em escala {K}, com LIDAR 2D. GERADO por tools/gerar_wanda.py --
    nao edite a mao. Feito para a fase 4 da CBR 2026, cujo labirinto tem
    janelas de 0.60 m.
  </description>
</model>
""", encoding="utf-8")

    # ── O relatorio, que e metade do valor do script ────────────────────────
    braco = 0.174 * K * math.sqrt(2)
    meia_helice = 0.2792307692307692 / 2 * K
    envergadura = 2 * (braco + meia_helice)
    empuxo_por_rotor = MASSA_ALVO * 9.81 / 4
    rot_pairar = math.sqrt(empuxo_por_rotor / (MOTOR_CONST_X500 * RAZAO_MASSA))

    print(f"  {NOME}: escala {K}, massa {MASSA_ALVO} kg")
    print(f"  ENVERGADURA {envergadura:.3f} m   (x500: 0.772 m)")
    print(f"  rotacao de pairar {rot_pairar:.0f} de {MAX_ROT_X500:.0f} rad/s "
          f"({100 * rot_pairar / MAX_ROT_X500:.0f}% do maximo)")
    print(f"  escrito em {destino}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
