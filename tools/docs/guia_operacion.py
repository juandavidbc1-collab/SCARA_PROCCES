#!/usr/bin/env python3
"""
Genera Guia_operacion_SCARA_ROS2.pdf (guía de cómo correr el sistema) con ReportLab.

    python3 tools/docs/guia_operacion.py            (necesita: pip install reportlab, fuentes DejaVu)

Edita el contenido aquí y vuelve a generar el PDF; así la guía no se desactualiza respecto al firmware.
"""
import os

from reportlab.graphics.shapes import Drawing, Line, Polygon, Rect, String
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (KeepTogether, Paragraph, Preformatted, SimpleDocTemplate, Spacer, Table,
                                TableStyle)

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "Guia_operacion_SCARA_ROS2.pdf")
DIR_FW = "~/Trabajos/Robotica/Robot/scara_fw_microros/scara_fw"

# ---------------------------------------------------------------- fuentes y estilos
FONT_DIRS = ["/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/dejavu", "/usr/share/fonts/TTF"]


def _font(name, file):
    for d in FONT_DIRS:
        p = os.path.join(d, file)
        if os.path.exists(p):
            pdfmetrics.registerFont(TTFont(name, p))
            return True
    return False


if _font("Sans", "DejaVuSans.ttf") and _font("Sans-B", "DejaVuSans-Bold.ttf") and _font("Mono", "DejaVuSansMono.ttf"):
    pdfmetrics.registerFontFamily("Sans", normal="Sans", bold="Sans-B", italic="Sans", boldItalic="Sans-B")
    SANS, SANS_B, MONO = "Sans", "Sans-B", "Mono"
else:  # sin DejaVu: fuentes base (sin algunos símbolos)
    SANS, SANS_B, MONO = "Helvetica", "Helvetica-Bold", "Courier"

NAVY = colors.HexColor("#1f3a5f")
BLUE = colors.HexColor("#2f80c4")
VIOLET = colors.HexColor("#6b4fc8")
DARK = colors.HexColor("#1e2b45")
ZEBRA = colors.HexColor("#eef3fa")
GRID = colors.HexColor("#c9d3e3")
GREY = colors.HexColor("#5b6472")

ST = {
    "title": ParagraphStyle("title", fontName=SANS_B, fontSize=24, leading=29, textColor=NAVY, spaceAfter=2),
    "subtitle": ParagraphStyle("subtitle", fontName=SANS, fontSize=11.5, leading=15, textColor=GREY),
    "author": ParagraphStyle("author", fontName=SANS, fontSize=8.5, leading=12, textColor=GREY, spaceBefore=6, spaceAfter=8),
    "h1": ParagraphStyle("h1", fontName=SANS_B, fontSize=15, leading=19, textColor=NAVY, spaceBefore=12, spaceAfter=6),
    "h2": ParagraphStyle("h2", fontName=SANS_B, fontSize=11, leading=14, textColor=BLUE, spaceBefore=8, spaceAfter=3),
    "body": ParagraphStyle("body", fontName=SANS, fontSize=9.3, leading=13, alignment=TA_LEFT, spaceAfter=4),
    "small": ParagraphStyle("small", fontName=SANS, fontSize=8, leading=10.5, textColor=GREY, spaceAfter=4),
    "cell": ParagraphStyle("cell", fontName=SANS, fontSize=8.1, leading=10.4),
    "cellb": ParagraphStyle("cellb", fontName=SANS_B, fontSize=8.1, leading=10.4),
    "head": ParagraphStyle("head", fontName=SANS_B, fontSize=8.3, leading=10.5, textColor=colors.white),
    "code": ParagraphStyle("code", fontName=MONO, fontSize=7.4, leading=9.4, leftIndent=8, textColor=colors.HexColor("#222222"),
                           spaceBefore=2, spaceAfter=6),
}


def P(t, s="body"):
    return Paragraph(t, ST[s])


def code(t):
    return Preformatted(t.strip("\n"), ST["code"])


def mono(t):
    return "<font name='%s'>%s</font>" % (MONO, t)


def table(rows, widths, head=True, first_bold=False, zebra=True):
    data = []
    for i, r in enumerate(rows):
        row = []
        for j, c in enumerate(r):
            if isinstance(c, str):
                stl = "head" if (head and i == 0) else ("cellb" if (first_bold and j == 0) else "cell")
                row.append(Paragraph(c, ST[stl]))
            else:
                row.append(c)
        data.append(row)
    t = Table(data, colWidths=widths, repeatRows=1 if head else 0)
    style = [("VALIGN", (0, 0), (-1, -1), "TOP"), ("GRID", (0, 0), (-1, -1), 0.4, GRID),
             ("LEFTPADDING", (0, 0), (-1, -1), 5), ("RIGHTPADDING", (0, 0), (-1, -1), 5),
             ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3)]
    if head:
        style.append(("BACKGROUND", (0, 0), (-1, 0), DARK))
    if zebra:
        for i in range(1 if head else 0, len(data)):
            if i % 2 == 0:
                style.append(("BACKGROUND", (0, i), (-1, i), ZEBRA))
    t.setStyle(TableStyle(style))
    return t


def K(*fl):
    """Mantiene juntos (en la misma página) un título con su contenido."""
    return KeepTogether(list(fl))


def checklist(items):
    return table([["[ ]", P(i, "cell")] for i in items], [0.9 * cm, 16.6 * cm], head=False, zebra=False)


# ---------------------------------------------------------------- diagrama de arquitectura
def arquitectura():
    W, H = 17.5 * cm, 4.3 * cm
    d = Drawing(W, H)
    boxes = [("PC con Ubuntu", BLUE, ["scara_panel (gráficas)", "scara_client · RViz", "ROS 2 Jazzy"]),
             ("Agente micro-ROS", VIOLET, ["Docker: micro-ros-agent", "traduce USB serie", "y ROS 2"]),
             ("ESP32 (robot)", DARK, ["nodo /scara/scara_esp32", "IK en C · control 1 kHz", "observadores 100 Hz"])]
    bw, bh, gap = 4.6 * cm, 3.6 * cm, (W - 3 * 4.6 * cm) / 2
    for i, (title, col, lines) in enumerate(boxes):
        x = i * (bw + gap)
        d.add(Rect(x, 0.2 * cm, bw, bh, rx=8, ry=8, fillColor=colors.HexColor("#f4f7fc"), strokeColor=col, strokeWidth=1.4))
        d.add(Rect(x, 0.2 * cm + bh - 0.8 * cm, bw, 0.8 * cm, rx=8, ry=8, fillColor=col, strokeColor=col))
        d.add(String(x + bw / 2, 0.2 * cm + bh - 0.55 * cm, title, fontName=SANS_B, fontSize=9.5, fillColor=colors.white, textAnchor="middle"))
        for k, ln in enumerate(lines):
            d.add(String(x + bw / 2, 0.2 * cm + bh - 1.45 * cm - k * 0.62 * cm, ln, fontName=SANS, fontSize=7.8, textAnchor="middle"))
    for i, lab in enumerate(("ROS 2", "USB / WiFi")):
        x0 = (i + 1) * bw + i * gap + 0.12 * cm
        x1 = x0 + gap - 0.24 * cm
        y = 0.2 * cm + bh / 2
        d.add(Line(x0, y, x1, y, strokeColor=colors.black, strokeWidth=1))
        for xa, s in ((x0, 1), (x1, -1)):
            d.add(Polygon([xa, y, xa + s * 6, y + 3.5, xa + s * 6, y - 3.5], fillColor=colors.black, strokeColor=colors.black))
        d.add(String((x0 + x1) / 2, y + 6, lab, fontName=SANS, fontSize=7.2, textAnchor="middle"))
    return d


# ---------------------------------------------------------------- contenido
def contenido():
    s = []
    s += [P("Guía de operación del robot SCARA", "title"),
          P("ROS 2 Jazzy + micro-ROS en Ubuntu | Panel de control, motores y observadores", "subtitle"),
          P("Juan David Botero Cubides - Mecatrónica, Universidad EIA", "author")]

    s += [P("1. Cómo está armado el sistema", "h1"), arquitectura(), Spacer(1, 4),
          P("El control de bajo nivel (lazo de 1 kHz, PID, cinemática inversa, seguridad y los <b>observadores de Luenberger y "
            "Kalman</b>) corre <b>dentro del ESP32</b>, en C. ROS 2 solo envía consignas y recibe el estado y las estimaciones. "
            "El ESP32 se conecta al PC por el cable USB: el <b>agente micro-ROS</b> lo convierte en un nodo más de ROS 2. "
            "El <b>panel de control</b> (" + mono("scara_panel.py") + ") muestra todo en gráficas y permite manejar el robot.")]

    s += [P("2. Antes de empezar (lista de chequeo)", "h1"), checklist([
        "ESP32 conectado por USB y placa de potencia alimentada.",
        "Ningún monitor serie abierto (pio device monitor, Arduino, etc.): el puerto es de uno solo.",
        "Robot despejado de cables y objetos, y el corte de alimentación al alcance de la mano.",
        "Puerto detectado: ejecuta " + mono("ls /dev/ttyUSB*") + " y debe aparecer /dev/ttyUSB0.",
        "Firmware en modo micro-ROS ya cargado (solo se repite si cambias algo del firmware; ver sección 11).",
        "Una sola vez: " + mono("sudo apt install python3-pyqt5 python3-pyqtgraph") + " (panel con gráficas).",
        "Archivos en " + mono(DIR_FW + "/ros2/") + ": scara_panel.py, scara_common.py, scara_client.py, "
        "scara_teleop_ros.py, scara_rviz.py, scara_rviz.launch.py, scara.urdf, scara.rviz."])]

    s += [P("3. Paso a paso para iniciar", "h1"),
          P("Abre las terminales en este orden. Las terminales nuevas ya cargan ROS 2 porque tu " + mono("~/.bashrc") +
            " hace el " + mono("source") + ". Deja abiertas las terminales 1 y 2 durante toda la demostración."),
          K(P("Terminal 1 - El agente micro-ROS (puente con el ESP32)", "h2"),
            code("docker run -it --rm -v /dev:/dev --privileged --net=host \\\n"
                 "    microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200 -v6")),
          P("Después pulsa el botón <b>EN</b> del ESP32 para reiniciarlo. <b>Qué imprime:</b> mientras el robot se conecta, líneas "
            "de información; ya conectado, un flujo continuo de líneas " + mono("debug") + " (es el tráfico del robot, es normal que "
            "no pare). Si eso aparece, la comunicación está viva."),
          code("[...] debug | SerialAgentLinux.cpp | recv_message | [==>> SER <<==] | client_key: 0x2AECB338 ...\n"
               "[...] debug | DataWriter.cpp       | write        | [** <<DDS>> **]  | client_key: 0x00000001 ..."),
          K(P("Terminal 2 - El panel de control", "h2"),
            code("cd " + DIR_FW + "\npython3 ros2/scara_panel.py")),
          P("<b>Qué se ve:</b> la ventana del panel. Arriba a la izquierda debe decir <b>● ESP32 conectado</b> (verde) y el modo "
            "(OFF al encender). Si dice <i>sin datos del ESP32</i>, revisa la terminal 1. Sin robot puedes practicar con "
            + mono("python3 ros2/scara_panel.py --demo") + " (simula el ESP32 con el modelo identificado)."),
          K(P("Terminal 3 (opcional) - RViz", "h2"),
            code("cd " + DIR_FW + "\nros2 launch ros2/scara_rviz.launch.py")),
          P("Se abre RViz con la vista desde arriba, los dos círculos verdes del espacio de trabajo y el robot."),
          K(P("Terminal 4 - Comprobar que todo está conectado", "h2"),
            code("ros2 node list\nros2 topic list\nros2 topic echo /scara/state --once")),
          K(P("<b>Qué imprime:</b>"),
            code("/scara/scara_esp32      <- el robot (nodo que corre en el ESP32)\n/scara_panel\n/scara_rviz  /robot_state_publisher  /rviz\n\n"
               "/joint_states  /scara/cmd  /scara/ee_position  /scara/event  /scara/goto\n"
               "/scara/jog     /scara/markers  /scara/obs  /scara/power  /scara/state\n\n"
               "data: -0.0, 0.0, 190.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...   (19 numeros; ver la seccion 9)")),
          P("Antes del Home el estado es q1 = 0, q2 = 0 y x = 190: no significa nada físico todavía, porque los encoders "
            "arrancan en cero al encender. <b>Si /scara/scara_esp32 no aparece, el agente no está conectado</b> (sección 10).")]

    s += [K(P("4. Demostración sugerida (con el panel)", "h1"),
          P("El <b>Home es obligatorio</b> después de cada encendido (sin Home el firmware rechaza los movimientos).", "body")),
          table([["#", "Acción en el panel", "Qué se ve / qué decir"],
                 ["1", "Botón <b>HOME</b>.", "Cada eje toca sus dos topes, mide su recorrido y vuelve. Home J1 ✓ y Home J2 ✓ en el encabezado."],
                 ["2", "Mover a una consigna: x = 120, y = 80, z = 0 → <b>Ir a (x, y, z)</b>.",
                  "La cinemática inversa se calcula en el ESP32. Evento #LLEGO con el error medido con encoders."],
                 ["3", "x = 400, y = 0 → Ir.", "#ERROR: inalcanzable (alcance de 50 a 190 mm). Validación de consignas."],
                 ["4", "q1 = 20, q2 = 90 → <b>Ir a ángulos</b>.", "Consigna articular. En la pestaña <b>Posición</b> se ve el brazo y la estela."],
                 ["5", "<b>Posición · flechas</b>: mantener ESPACIO (o MANTENER) + flechas.", "Manual con dead-man: al soltar, se detiene."],
                 ["6", "<b>Potencia · deslizadores</b>: M1 en ▶ 30 %, mantener el dead-man.",
                  "Lazo abierto. J2 (0 %) mantiene su posición. Al soltar frena y se queda donde paró."],
                 ["7", "Habilitar <b>M3</b> (interruptor), deslizador ▶ 40 %, mantener el dead-man.", "El atornillador gira; sus observadores aparecen en Observadores → M3."],
                 ["8", "Deshabilitar <b>M2</b> e Ir a ángulos.", "Solo se mueve J1; M2 queda libre (se puede mover a mano y no pierde el Home)."],
                 ["9", "Pestaña <b>Observadores</b>, M1: mover y luego <b>Reiniciar observadores</b>.",
                  "Medida vs Luenberger vs Kalman a priori/a posteriori; corriente estimada; σ y K convergen (K<sub>w</sub> ≈ 0,266)."],
                 ["10", "<b>Grabar CSV</b> durante una prueba.", "Archivo scara_AAAAMMDD_HHMMSS.csv con todas las estimaciones (20 Hz)."]],
                [0.9 * cm, 7.1 * cm, 9.5 * cm])]

    s += [P("5. Qué imprime cada comando (salidas esperadas)", "h1"),
          P("Los mismos comandos sirven en la línea de comandos del panel, con " + mono("python3 ros2/scara_client.py cmd \"...\"") +
            " o con " + mono("ros2 topic pub /scara/cmd ...") + "."),
          K(P("home", "h2"),
            code("#OK: home solicitado\n#HOME: iniciando. Eje 2: primero un tope, luego el otro, y vuelve a donde estaba.\n"
               "#HOME: eje 2 referenciado en el tope MIN (q = -143.00 deg). Ahora el tope contrario...\n"
               "#HOME: eje 2 recorrido medido: de -143.00 a 132.76 deg (total 275.76 deg). Limites por software: -140.02 a 129.78 deg.\n"
               "#HOME: ahora el eje 1, buscando su primer tope...\n#HOME: eje 1 referenciado en el tope MIN (q = -108.00 deg). ...\n"
               "#HOME: terminado. Volviendo a la posicion inicial (q1 = -1.12, q2 = 0.00 deg)...\n#LLEGO: q1=-1.121 q2=0.000 deg | medido x=189.96 y=-3.72 mm")),
          K(P("goto 120 80 0  y  goto 400 0", "h2"),
            code("#INFO: cinematica inversa (120.0, 80.0, 0.0) -> q1 = 4.78 deg, q2 = 84.88 deg (codo +1). Moviendo...\n"
               "#LLEGO: objetivo x=120.00 y=80.00 z=0.00 | medido x=120.15 y=79.73 | error ex=0.153 ey=-0.271 mm | q1=4.651 q2=84.900 deg\n"
               "#ERROR: no se puede ir a (400.0, 0.0): inalcanzable (fuera del espacio de trabajo). Alcance: 50.0 a 190.0 mm.")),
          K(P("Motores, observadores y otros", "h2"),
            code("> motor 3 on      -> #OK: motor 3 habilitado          y   #MOTOR: M3 HABILITADO.\n"
               "> motor 2 off     -> #OK: motor 2 deshabilitado       y   #MOTOR: M2 deshabilitado (libre).\n"
               "> motors          -> #INFO: motores: M1 ON, M2 off, M3 ON\n"
               "> goto 120 80     -> #ERROR: goto necesita J1 y J2 habilitados (M2 deshabilitado: usa 'motor 2 on').\n"
               "> est 1           -> #INFO: M1 polos 0.250+-0.250j L=[0.2436, 0.412] | Kalman sig_i=10.0 mA sig_w=0.30 rpm\n"
               "                     sig_y=0.81 rpm | V=1.000*12*duty-0.80\n"
               "> luen 1 0.1 0.1  -> #OK: M1 Luenberger con polos 0.100 +- 0.100j\n"
               "                     #EST: M1 Luenberger polos 0.100 +/- 0.100j -> L = [0.036732, 0.71198]\n"
               "> kal 1 10 0.3 0.81 -> #OK: M1 Kalman sig_i=10.0 mA sig_w=0.30 rpm sig_y=0.81 rpm\n"
               "> est reset       -> #OK: observadores reiniciados\n"
               "> gain            -> #INFO: J1 kp=5350 ki=18500 kd=295 kff=85 fric=70 | J2 kp=5000 ki=3000 kd=180 kff=80 fric=70\n"
               "> stop / off / clear / status  (como antes; 'status' ahora incluye  motores:12-)\n"
               "(stop con el dead-man de potencia presionado) -> #INFO: potencia manual bloqueada: suelta el dead-man ..."))]

    s += [P("6. El panel de control (scara_panel.py)", "h1"),
          P("Se abre con " + mono("python3 ros2/scara_panel.py") + " (" + mono("--demo") + " sin robot). Reemplaza al panel Tk; "
            + mono("scara_teleop_ros.py") + " queda como respaldo liviano."),
          table([["Zona", "Qué hace"],
                 ["Encabezado", "Conexión con el ESP32, modo, Home J1/J2, potencia manual activa, corriente estimada de la fuente, falla y los 4 finales de carrera. "
                                "Botones HOME, DETENER, BORRAR FALLA y APAGAR."],
                 ["Tarjetas de motor", "Interruptor para <b>habilitar/deshabilitar</b>; velocidad y corriente del motor (Kalman); PWM real; "
                                       "<b>deslizador de potencia de 0 a 100 %</b> con sentido ◀ ▶; debajo, el % de PWM que se envía. "
                                       "Clic en el valor = 0."],
                 ["Control manual", "<b>Posición</b> (flechas → /scara/jog) o <b>Potencia</b> (deslizadores → /scara/power). "
                                    "Dead-man: botón verde MANTENER o ESPACIO. Los botones de flecha también funcionan manteniéndolos."],
                 ["Mover a una consigna", "x, y, z en mm (codo +1 / −1) o q1, q2 en grados. En Posición, un clic en el plano elige (x, y)."],
                 ["Pestaña Observadores", "Por motor: velocidad medida, Luenberger, Kalman a priori y a posteriori; corriente estimada; "
                                          "incertidumbre √P; ganancia de Kalman. Botón Reiniciar observadores."],
                 ["Pestaña Motores", "Velocidad, corriente, PWM y voltaje del modelo de los 3 motores juntos, y la corriente que debería marcar la fuente."],
                 ["Pestaña Posición", "Vista superior: espacio de trabajo, brazo, estela y consigna; q contra q de referencia."],
                 ["Pestaña Sintonía", "PID por eje (leer/aplicar), polos del Luenberger y σ del Kalman por motor, mapa de voltaje "
                                      "V = g·12·duty − v0 (valores del L298N del laboratorio o del TB6612), escala de los deslizadores por motor "
                                      "(arranque y tope) y corriente de la parte lógica."],
                 ["Registro", "Eventos con colores (verde #LLEGO/#OK, rojo #ERROR/#FALLA, ámbar #MOTOR/#EST/#HOME), línea de "
                              "comandos con historial, ventana de tiempo, pausa y Grabar CSV."],
                 ["Seguridad", "Escribir en un cuadro o perder el foco suelta el dead-man. Clic fuera de los cuadros (o Esc) "
                               "devuelve el teclado al robot."]],
                [3.6 * cm, 13.9 * cm], first_bold=True),
          Spacer(1, 6),
          table([["Tecla", "Acción", "Tecla", "Acción"],
                 ["ESPACIO (mantener)", "dead-man", "h", "Home"],
                 ["← →", "J1 − / +", "s · Esc", "Detener"],
                 ["↓ ↑", "J2 − / +", "o", "Apagar motores"],
                 ["r", "Reiniciar observadores", "c", "Borrar falla"]],
                [3.6 * cm, 5.1 * cm, 3.6 * cm, 5.2 * cm])]

    s += [K(P("7. Motores y potencia manual", "h1"),
          table([["Regla", "Detalle"],
                 ["Habilitar / deshabilitar", "Interruptor del panel, " + mono("motor j on|off") + " o " + mono("motors a b c") +
                  ". Al encender: M1 y M2 habilitados, M3 deshabilitado."],
                 ["Motor deshabilitado", "Driver libre (sin PWM). El encoder sigue contando: no pierde el Home. No entra al Home, al "
                                         "PID ni a la supervisión de fallas."],
                 ["goto / ang", "goto necesita J1 y J2 habilitados. ang mueve solo los ejes habilitados."],
                 ["Potencia manual", "Solo mientras llega el dead-man (25 Hz; sin latidos 300 ms → 0). Ejes con potencia: lazo "
                                     "abierto. Ejes con 0 %: mantienen posición. Al soltar: frenan y se quedan donde paran."],
                 ["Atornillador (M3)", "Solo se mueve con potencia manual; funciona en paralelo al brazo, incluso con el brazo en OFF."],
                 ["Escala 0–100 %", "0 % = parado (mantiene posición); 1 % = <i>arranque</i> (PWM con el que apenas gira); 100 % = "
                                    "<i>tope</i> del motor. Por defecto: arranque 7 %; tope 60 % M1, 30 % M2, 60 % M3. Se cambian en "
                                    "Sintonía y el panel los recuerda. Por /scara/power sigue viajando % de PWM real."],
                 ["Límites", "Tope 63 % de PWM, rampa de 0,34 s, sin empujar contra un final de carrera, límites por software con Home."],
                 ["Corriente de la fuente", "La de las tarjetas es la del motor. La fuente marca I<sub>lógica</sub> + Σ |i<sub>motor</sub>| · duty: "
                                            "la lógica (ESP32, reguladores, encoders, driver) se mide con los motores APAGADOS y se escribe "
                                            "en Sintonía; con PWM, el motor solo toma corriente de la fuente el tiempo encendido."],
                 ["Bloqueo", "Detener, Apagar o una falla bloquean la potencia hasta soltar y volver a presionar el dead-man."]],
                [3.6 * cm, 13.9 * cm], first_bold=True))]

    s += [P("8. Observadores (Luenberger y Kalman)", "h1"),
          P("Corren en el ESP32 para los 3 motores a 100 Hz (Ts = 10 ms), con el modelo identificado en el Laboratorio 4. "
            "Miden solo la velocidad (diferencia de cuentas del encoder en 10 ms) y estiman velocidad y corriente."),
          code("La di/dt = V - Ra i - Kv w      J dw/dt = Kv i - B w      y = w   (eje de salida)\n"
               "Luenberger:  x^(k) = A x^(k-1) + B u(k-1) + L [ y(k-1) - C x^(k-1) ]       L por Ackermann\n"
               "Kalman:      x-(k) = A x+(k-1) + B u(k-1),  P- = A P+ A' + Q,  K = P- C'(C P- C' + R)^-1,\n"
               "             x+(k) = x- + K (y(k) - C x-),  P+ = (I - K C) P-"),
          table([["Parámetro", "Valor por defecto", "Cambiar en vivo"],
                 ["Modelo", "Ra = 50 Ω, La = 0,060 H, Kv = 0,1671 V·s/rad, B = 2,79·10⁻³, J = 3,63·10⁻⁴ (eje de salida)", "config.h (MOT_*)"],
                 ["Luenberger", "polos 0,25 ± 0,25j → L = [0,2436; 0,4120]", mono("luen j re im")],
                 ["Kalman", "σi = 10 mA, σw = 0,30 rpm, σy = 0,81 rpm (R = 0,655 rpm²) → K<sub>w</sub> ≈ 0,266", mono("kal j σi σw σy")],
                 ["Voltaje", "V = g·(12 V·duty) − v0; TB6612: g = 1, v0 = 0,8 V · L298N (lab): g = 0,9782, v0 = 1,197 V", mono("vmap g v0")]],
                [2.6 * cm, 10.4 * cm, 4.5 * cm], first_bold=True),
          Spacer(1, 4),
          P("<b>Cómo leer las gráficas:</b> el Kalman a posteriori (cian) sigue a la medida sin el ruido de cuantización; el "
            "a priori (naranja) es lo que dice solo el modelo; el Luenberger (verde) es más ruidoso en la corriente. Tras "
            "<i>Reiniciar observadores</i>, σ arranca alta y baja, y K<sub>w</sub> se estabiliza en ≈ 0,266 (en estado estable el filtro "
            "le cree ~27 % a la medida y ~73 % al modelo). Con Ra = 50 Ω la corriente estimada casi no reacciona a la carga: "
            "el Kalman casi no la corrige (K<sub>i</sub> ≈ 0).", "body")]

    s += [K(P("9. Qué hace cada nodo", "h1"),
          table([["Nodo", "Dónde corre", "Qué hace", "Publica / muestra"],
                 ["/scara/scara_esp32", "ESP32", "Recibe /scara/goto, /scara/jog, /scara/power y /scara/cmd; IK, control en lazo "
                  "cerrado, motores habilitados, potencia manual, finales de carrera, dead-man y observadores.",
                  "/scara/state, /scara/obs, /scara/ee_position, /scara/event"],
                 ["micro_ros_agent", "PC (Docker)", "Puente: traduce lo que el ESP32 envía por USB a tópicos de ROS 2, y al revés.", "Líneas debug"],
                 ["scara_panel", "PC (Python)", "Panel PyQt5: gráficas, motores, dead-man (25 Hz), consignas, sintonía, CSV.", "La ventana"],
                 ["scara_client", "PC (Python)", "Un comando por ejecución; espera el evento final.", "Eventos"],
                 ["scara_teleop", "PC (Python)", "Panel Tk anterior (respaldo, sin gráficas).", "Registro"],
                 ["scara_rviz / rviz2", "PC", "Convierte /scara/state en /joint_states y dibuja el espacio de trabajo y la estela.", "RViz"]],
                [3.2 * cm, 2.1 * cm, 7.8 * cm, 4.4 * cm], first_bold=True)),
          P("Tópicos", "h2"),
          table([["Tópico", "Tipo", "Sentido", "Contenido"],
                 [mono("/scara/goto"), "geometry_msgs/Point", "PC → ESP32", "Consigna x, y, z en mm (z = altura fija)."],
                 [mono("/scara/jog"), "geometry_msgs/Vector3", "PC → ESP32", "25 Hz: x = dead-man, y = dir J1, z = dir J2."],
                 [mono("/scara/power"), "Float32MultiArray", "PC → ESP32", "<b>Nuevo.</b> 25 Hz: [dead-man, %M1, %M2, %M3]."],
                 [mono("/scara/cmd"), "std_msgs/String", "PC → ESP32", "home, stop, off, clear, goto, ang, elbow, gain, pulse, motor, "
                  "motors, est, luen, kal, vmap, status, help."],
                 [mono("/scara/state"), "Float32MultiArray", "ESP32 → PC", "19 números a 20 Hz (tabla de abajo)."],
                 [mono("/scara/obs"), "Float32MultiArray", "ESP32 → PC", "<b>Nuevo.</b> 38 números a 20 Hz: observadores de los 3 motores."],
                 [mono("/scara/ee_position"), "geometry_msgs/Point", "ESP32 → PC", "Posición medida del efector en mm."],
                 [mono("/scara/event"), "std_msgs/String", "ESP32 → PC", "#OK, #INFO, #HOME, #LLEGO, #ERROR, #FALLA, #MOTOR, #EST."]],
                [3.4 * cm, 3.7 * cm, 2.3 * cm, 8.1 * cm]),
          K(P("Los 19 datos de /scara/state", "h2"),
          table([["Pos.", "Significado", "Pos.", "Significado"],
                 ["0, 1", "q1, q2 en grados", "11, 12", "PWM de M1, M2"],
                 ["2, 3", "x, y del efector en mm", "13", "Motores habilitados (bits: 1 = M1, 2 = M2, 4 = M3)"],
                 ["4", "Modo: 0 OFF, 1 HOMING, 2 HOLD, 3 MOVE, 4 JOG, 5 PULSE, 6 FAULT, 7 POWER", "14", "PWM de M3"],
                 ["5", "Falla: 0 ninguna, 1 seguimiento, 2 trabado, 3 final de carrera, 4 tiempo de Home, 5 MIN y MAX", "15", "Potencia manual activa"],
                 ["6, 7", "Home hecho en J1, J2", "16, 17, 18", "Velocidad medida M1, M2, M3 [rpm]"],
                 ["8", "Finales de carrera (bits): 1 J1 min, 2 J1 max, 4 J2 min, 8 J2 max", "", ""],
                 ["9, 10", "Referencia interna de q1, q2 en grados", "", ""]],
                [1.4 * cm, 8.1 * cm, 1.9 * cm, 6.1 * cm])),
          K(P("/scara/obs: [secuencia, banderas, 12 datos por motor × 3]", "h2"),
          P("Por motor: velocidad medida, Luenberger, Kalman a priori, Kalman a posteriori [rpm]; corriente Luenberger, Kalman a "
            "priori, Kalman a posteriori [A]; σ corriente [A]; σ velocidad [rpm]; K<sub>w</sub>; K<sub>i</sub>; voltaje del modelo [V]. "
            "Banderas: bit m = diseño válido del motor m; bit m+3 = el motor tiene encoder.", "body"))]

    s += [P("10. Errores frecuentes y cómo resolverlos", "h1"),
          table([["Síntoma / mensaje", "Causa probable", "Solución"],
                 ["<b>El panel dice 'sin datos del ESP32'</b> / No llega estado", "El agente no corre, puerto o baudios distintos, "
                  "monitor serie abierto, o el ESP32 no se reinició.", "Revisa la terminal 1. Cierra monitores. Confirma -b 115200 y "
                  "/dev/ttyUSB0. Pulsa EN."],
                 ["<b>Falta PyQt5 o pyqtgraph</b>", "No están instaladas las librerías del panel.", mono("sudo apt install python3-pyqt5 python3-pyqtgraph")],
                 ["docker: command not found / permission denied", "Docker no instalado o usuario fuera del grupo docker.",
                  mono("sudo apt install docker.io") + ", " + mono("sudo usermod -aG docker $USER") + " y reinicia la sesión."],
                 ["<b>#ERROR: primero ejecuta HOME</b>", "Se encendió el robot o se borró una falla.", "Pulsa HOME."],
                 ["<b>#ERROR: goto necesita J1 y J2 habilitados</b>", "Un motor del brazo está deshabilitado.", "Actívalo con su interruptor o " + mono("motor 2 on") + "."],
                 ["<b>#INFO: potencia manual bloqueada</b>", "Detener/Apagar/falla con el dead-man presionado.", "Suelta el dead-man y vuelve a presionarlo."],
                 ["M3 no gira con potencia", "M3 arranca deshabilitado, o no estás en modo Potencia.", "Interruptor de M3, modo Potencia y mantener el dead-man."],
                 ["Velocidad medida de M3 siempre 0", "Pines del encoder 3 (J8) distintos a los de config.h.", "Corrige PIN_ENC3_A/B o pon M3_TIENE_ENCODER 0."],
                 ["<b>#FALLA: final de carrera inesperado</b> al iniciar Home", "El brazo quedó apoyado en un tope o un interruptor da señal falsa.",
                  "Apagar, mover el brazo al centro, mirar los topes en el encabezado (todos apagados), Borrar falla y Home."],
                 ["<b>#FALLA: error de seguimiento</b> o <b>motor trabado</b>", "Algo frena el motor, signo invertido o ganancias.",
                  "Libera el brazo, Borrar falla, repite Home. Si vuelve: pulse 1 / pulse 2 y revisa ganancias (respaldo: 5000 3000 180 80 70)."],
                 ["<b>#FALLA: tiempo de Home agotado</b>", "PWM de búsqueda bajo o brazo trabado.", "Revisa que nada estorbe; sube HOME_DUTY (170) en config.h."],
                 ["<b>#ERROR: no se puede ir a (x, y): inalcanzable</b>", "Fuera de 50 a 190 mm o singular.", "Punto dentro de los círculos o el otro codo."],
                 ["Un eslabón muy agresivo (vibra, busca la posición, PWM alto estando quieto)", "Ganancias altas para ese eje (sobre todo ki).",
                  "Sintonía: baja ki y luego kd, Aplicar (dura hasta reiniciar). Para dejarlo fijo: src/config.c. J2 ya viene con 5000 3000 180 80 70."],
                 ["<b>#ERROR: no llego a la consigna a tiempo</b>", "El brazo se detuvo justo fuera de la tolerancia (fricción).", "Revisa ganancias (fric, ki) o repite."],
                 ["El robot se detiene a ratos en manual", "Se pierden latidos del dead-man (300 ms).", "Comportamiento seguro; cierra programas pesados, revisa el USB."],
                 ["Failed to resolve component 'micro_ros_espidf_component'", "El zip no trae el componente micro-ROS (es otro repositorio).",
                  "git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component"],
                 ["Las gráficas van 'a saltos'", "Ancho de banda a 115200 baudios.", "Sube a 460800: " + mono("uros_select.py serial --baud 460800") + " y -b 460800 en el agente."],
                 ["RViz: el robot no aparece / ejes gigantes", "scara_rviz no corre / display TF activo.", "ros2 node list debe mostrar /scara_rviz; desmarca TF."],
                 [mono("Could not exclusively lock port"), "Otro programa tiene el puerto.", mono("fuser -v /dev/ttyUSB0") + " y cierra ese proceso."]],
                [5.0 * cm, 5.4 * cm, 7.1 * cm])]

    s += [K(P("11. Si hay que volver a compilar el firmware", "h1"),
          P("Solo hace falta si cambias " + mono("config.c") + " o " + mono("config.h") + " (pines del encoder 3, motores habilitados "
            "al arrancar, ganancias, modelo, observadores). En una carpeta nueva (zip o clon) primero hay que clonar el componente "
            "micro-ROS, que no viene incluido, y elegir el modo (esto también lo deja compatible con ESP-IDF 5.3):"),
          code("git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component\n"
               "python3 tools/uros_select.py serial"),
          P("Compila siempre con un entorno limpio, sin ROS cargado:"),
          code("env -i HOME=\"$HOME\" TERM=\"$TERM\" \\\n"
               "  PATH=\"/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$HOME/.local/bin\" \\\n"
               "  bash --noprofile --norc -c \\\n"
               "  'cd " + DIR_FW + " && pio run -t upload'")),
          table([["Error al compilar", "Solución"],
                 [mono("Unknown boolean field \"use_gitignore\""), mono("python3 tools/uros_select.py serial") + " lo corrige solo; a mano: " + mono("sed -i '/use_gitignore/d' components/micro_ros_espidf_component/idf_component.yml")],
                 [mono("CMake 3.20 or higher is required"), "Enlazar el CMake del sistema: " + mono("ln -s /usr/bin/cmake ~/.platformio/packages/tool-cmake/bin/cmake")
                  + " (después de mover el original a cmake.orig)."],
                 [mono("Could not find ament_cmake") + " / errores raros de ROS", "Compilaste con ROS cargado: usa el comando env -i de arriba."],
                 [mono("*.obj: No such file") + " en micro_ros", "Borra components/micro_ros_espidf_component/{micro_ros_dev,micro_ros_src,libmicroros.a,include} y compila de nuevo."],
                 ["Se reinicia solo al conectar", "Sube CONFIG_SCARA_UROS_STACK (menuconfig)."]],
                [6.5 * cm, 11.0 * cm])]

    s += [KeepTogether([P("12. Al terminar", "h1"),
                        P("Cierra con Ctrl+C (o la ventana) en este orden: el panel, RViz y por último el agente (terminal 1). "
                          "Los motores quedan en HOLD; si cortas la alimentación de potencia se sueltan. Al volver a encender hay "
                          "que repetir el Home.")])]
    return s


def deco(canvas, doc):
    canvas.saveState()
    canvas.setFont(SANS, 7.5)
    canvas.setFillColor(GREY)
    canvas.setStrokeColor(GRID)
    canvas.line(2 * cm, 1.4 * cm, letter[0] - 2 * cm, 1.4 * cm)
    canvas.drawString(2 * cm, 1.0 * cm, "Guía de operación - Robot SCARA con ROS 2 / micro-ROS")
    canvas.drawRightString(letter[0] - 2 * cm, 1.0 * cm, "Página %d" % doc.page)
    canvas.restoreState()


def main():
    doc = SimpleDocTemplate(OUT, pagesize=letter, leftMargin=2 * cm, rightMargin=2 * cm, topMargin=1.7 * cm, bottomMargin=2 * cm,
                            title="Guía de operación del robot SCARA", author="Juan David Botero Cubides")
    doc.build(contenido(), onFirstPage=deco, onLaterPages=deco)
    print("escrito", OUT)


if __name__ == "__main__":
    main()
