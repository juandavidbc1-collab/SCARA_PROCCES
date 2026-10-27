#!/bin/sh
# Pruebas en el PC (Linux / WSL / MSYS2 con gcc). Ejecutar desde la raíz del proyecto:  sh tools/host_tests/run.sh
set -e
mkdir -p /tmp/scara_host
gcc -Wall -Wextra -O2 -Isrc tools/host_tests/test_kin_traj.c src/kinematics.c src/traj.c -lm -o /tmp/scara_host/t_kin && /tmp/scara_host/t_kin
gcc -O2 -Isrc tools/host_tests/sim_pid.c src/pid.c src/traj.c -lm -o /tmp/scara_host/sim_pid && /tmp/scara_host/sim_pid 5000 3000 180 80 60
# la simulación de todo el firmware necesita las cabeceras "falsas" de ESP-IDF de tools/host_tests/stubs
gcc -std=gnu11 -O2 -Wall -Wextra -Itools/host_tests/stubs -Isrc -Itools/host_tests tools/host_tests/sim_control.c src/estimator.c \
    src/config.c src/kinematics.c src/pid.c src/traj.c -lm -o /tmp/scara_host/sim_ctl && /tmp/scara_host/sim_ctl | grep -v '^  \['
gcc -std=gnu11 -O1 -Wall -Wextra -Itools/host_tests/stubs -Isrc tools/host_tests/test_comms.c src/config.c src/kinematics.c src/motion.c src/uros_cmd.c -lm -o /tmp/scara_host/t_comms && /tmp/scara_host/t_comms | grep -E '\[ok\]|FALLO|TODO OK|FALLOS'
# observadores (Luenberger + Kalman): diseño y comparación con los números del Laboratorio 4
gcc -std=gnu11 -O2 -Wall -Wextra -Isrc tools/host_tests/test_estimator.c src/estimator.c -lm -o /tmp/scara_host/t_est && /tmp/scara_host/t_est
# intérprete de comandos de ROS 2 + módulo compartido de movimiento
gcc -std=gnu11 -O1 -Wall -Wextra -Itools/host_tests/stubs -Isrc tools/host_tests/test_uros_cmd.c src/uros_cmd.c src/motion.c src/config.c src/kinematics.c -lm -o /tmp/scara_host/t_uros && /tmp/scara_host/t_uros | grep -E '\[ok\]|FALLO|TODO OK|FALLOS'
# la capa micro-ROS no se puede ejecutar en el PC; se comprueba que COMPILA contra cabeceras falsas con las firmas de rclc/rmw (jazzy)
gcc -fsyntax-only -std=gnu11 -Wall -Wextra -DFAKE_SERIAL -Itools/host_tests/stubs_uros -Itools/host_tests/stubs -Isrc src/uros_comms.c src/uros_serial_transport.c && echo "  [ok] uros_comms.c + transporte serial compilan (sintaxis)"
gcc -fsyntax-only -std=gnu11 -Wall -Wextra -Itools/host_tests/stubs_uros -Itools/host_tests/stubs -Isrc src/uros_comms.c && echo "  [ok] uros_comms.c compila con transporte UDP/WiFi (sintaxis)"
# módulo común de Python (formato de tópicos y estimador del modo demo); necesita numpy
if python3 -c "import numpy" 2>/dev/null; then python3 tools/host_tests/test_common.py; else echo "  (sin numpy: se omite test_common.py)"; fi
