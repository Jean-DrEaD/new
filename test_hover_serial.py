#!/usr/bin/env python3
"""
test_hover_serial.py  —  Testa comunicação USART3 com STM32 ONE_AXIS_USART_VARIANT
Protocolo real do firmware (zip USART3_ultima):
  START_FRAME = 0xABCD  (little-endian)
  Checksum    = XOR de todos os campos uint16/int16 exceto checksum
  Baud rate   = 500000 (500 kbps)  ← crítico, não 115200

Uso:
  pip install pyserial
  python test_hover_serial.py --port /dev/ttyUSB0
  python test_hover_serial.py --port COM5
  python test_hover_serial.py --port /dev/ttyUSB0 --test sweep
  python test_hover_serial.py --port /dev/ttyUSB0 --test interactive

NOTA: O campo SPEED contém o torque FFB (TANK_STEERING, single motor).
      O campo STEER deve ser sempre 0 no ONE_AXIS_USART_VARIANT.
"""

import serial, struct, time, threading, argparse, math, sys

# ---- Protocolo ----
START_FRAME   = 0xABCD
HOVER_BAUD    = 500000   # NÃO 115200 !
CMD_FMT       = '<HhhH'   # start(H) steer(h) speed(h) checksum(H) → 8 bytes
FB_FMT        = '<HhhhhhhHH'  # 18 bytes
CMD_SIZE      = struct.calcsize(CMD_FMT)   # 8
FB_SIZE       = struct.calcsize(FB_FMT)   # 18

assert CMD_SIZE == 8,  f"Esperado CMD_SIZE=8, obteve {CMD_SIZE}"
assert FB_SIZE  == 18, f"Esperado FB_SIZE=18, obteve {FB_SIZE}"

# CPR do encoder MT6701 ABZ @ 1024 PPR
ENCODER_CPR   = 4096   # 1024 PPR * 4 quadrature
ROTATION_MAX  = 12288  # ±12288 counts = ±3 voltas = ±1080 graus


def checksum_cmd(start, steer, speed):
    return start ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)

def checksum_fb(start, cmd1, cmd2, sR, sL, bat, temp, led):
    return (start             ^
            (cmd1  & 0xFFFF)  ^
            (cmd2  & 0xFFFF)  ^
            (sR    & 0xFFFF)  ^
            (sL    & 0xFFFF)  ^
            (bat   & 0xFFFF)  ^
            (temp  & 0xFFFF)  ^
            (led   & 0xFFFF))

def build_command(torque: int) -> bytes:
    """
    Monta pacote Command de 8 bytes.
    torque: valor de torque FFB [-1000, +1000] → campo SPEED
    steer:  sempre 0 (single-axis, TANK_STEERING)
    """
    torque = max(-1000, min(1000, torque))
    steer  = 0
    chk    = checksum_cmd(START_FRAME, steer, torque)
    return struct.pack(CMD_FMT, START_FRAME, steer, torque, chk)

def parse_feedback(data: bytes):
    """Parseia 18 bytes de Feedback. Retorna dict ou None."""
    if len(data) < FB_SIZE:
        return None
    fields = struct.unpack(FB_FMT, data[:FB_SIZE])
    start, cmd1, cmd2, sR, sL, bat, temp, led, chk = fields
    if start != START_FRAME:
        return None
    if chk != checksum_fb(start, cmd1, cmd2, sR, sL, bat, temp, led):
        return None
    # sL = enc_pos (posição encoder, não velocidade!)
    # Para GD32: firmware nega enc_pos → recebemos -enc_pos → negamos de volta
    degrees = (sL / ENCODER_CPR) * 360.0
    return {
        'cmd1':      cmd1,
        'cmd2':      cmd2,
        'speedR':    sR,           # RPM motor
        'enc_pos':   sL,           # posição encoder (counts), 0 = centro
        'enc_deg':   degrees,      # posição em graus
        'bat_v':     bat / 100.0,  # Tensão em V
        'temp_c':    temp,         # °C (inteiro direto neste firmware)
        'led':       led,
    }

def counts_to_degrees(counts: int) -> float:
    return (counts / ENCODER_CPR) * 360.0


class HoverTest:
    def __init__(self, port: str, baud: int = HOVER_BAUD):
        self.ser     = serial.Serial(port, baud, timeout=0.005)
        self.running = False
        self.lock    = threading.Lock()
        self.last_fb = None
        self.stats   = {'tx': 0, 'rx_ok': 0, 'rx_err': 0}

    def send(self, torque: int):
        """Envia comando de torque ao STM32."""
        pkt = build_command(torque)
        self.ser.write(pkt)
        with self.lock:
            self.stats['tx'] += 1

    def _rx_loop(self):
        prev = 0
        buf  = bytearray()
        while self.running:
            data = self.ser.read(64)
            if not data:
                continue
            for b in data:
                word = (b << 8) | prev
                if word == START_FRAME:
                    buf = bytearray([prev, b])
                elif len(buf) >= 2:
                    buf.append(b)
                    if len(buf) == FB_SIZE:
                        fb = parse_feedback(bytes(buf))
                        with self.lock:
                            if fb:
                                self.last_fb = fb
                                self.stats['rx_ok'] += 1
                            else:
                                self.stats['rx_err'] += 1
                        buf = bytearray()
                prev = b

    def start(self):
        self.running = True
        self.t = threading.Thread(target=self._rx_loop, daemon=True)
        self.t.start()

    def stop(self):
        self.running = False
        self.send(0)
        time.sleep(0.1)
        self.ser.close()

    def get_feedback(self):
        with self.lock:
            return self.last_fb

    def print_stats(self):
        s = self.stats
        print(f"  TX={s['tx']}  RX_ok={s['rx_ok']}  RX_err={s['rx_err']}")

    def print_feedback(self, fb):
        if not fb:
            return
        print(f"  enc_pos={fb['enc_pos']:+6d} cnt  "
              f"({fb['enc_deg']:+7.1f}°)  "
              f"RPM={fb['speedR']:+5d}  "
              f"bat={fb['bat_v']:.2f}V  "
              f"temp={fb['temp_c']}°C")

    # ---- Testes ----

    def test_ping(self, duration=5.0):
        """Envia torque=0 por N segundos. Confirma que o STM32 responde."""
        print(f"\n[PING] torque=0 por {duration:.0f}s — aguardando Feedback do STM32...")
        t0 = time.time()
        while time.time() - t0 < duration:
            self.send(0)
            time.sleep(0.02)   # 50 Hz (ciclo do firmware é 5ms = 200Hz)

        fb = self.get_feedback()
        self.print_stats()
        if self.stats['rx_ok'] == 0:
            print("\n  ⚠  NENHUM FEEDBACK RECEBIDO. Checklist:")
            print("     1. platformio.ini: default_envs = ONE_AXIS_USART_VARIANT")
            print("     2. Baud STM32: config.h #define USART3_BAUD 500000")
            print("     3. Baud Python: --baud 500000 (padrão)")
            print("     4. Wiring: ESP32/Arduino RX → STM32 PB10 (USART3 TX)")
            print("                ESP32/Arduino TX → STM32 PB11 (USART3 RX)")
            print("     5. GND comum entre ESP32 e placa hoverboard")
            print("     6. STM32 ligado (chave power ON)")
        else:
            print(f"\n  ✓  Comunicação OK — último feedback:")
            self.print_feedback(fb)

    def test_sweep_torque(self, max_t=500, step=50, delay=0.3):
        """Varre torque de -max a +max. Motor gira suavemente se tudo estiver correto."""
        print(f"\n[SWEEP] Torque ±{max_t}, step={step}, delay={delay}s por passo")
        print("  (Motor deve girar levemente — segure a roda!)")
        vals = (list(range(0, max_t + 1, step)) +
                list(range(max_t, -max_t - 1, -step)) +
                list(range(-max_t, 1, step)))
        for v in vals:
            self.send(v)
            fb = self.get_feedback()
            pos_str = f"{fb['enc_pos']:+6d}cnt ({fb['enc_deg']:+6.1f}°)" if fb else "aguardando..."
            print(f"  torque={v:+5d}  enc_pos={pos_str}")
            time.sleep(delay)
        self.send(0)
        print("  Sweep concluído.")
        self.print_stats()

    def test_center_spring(self, stiffness=3.0, duration=10.0):
        """
        Simula spring de centralização simples.
        torque = -stiffness * enc_pos / counts_per_degree
        Segure a roda e solte — ela deve voltar ao centro.
        """
        print(f"\n[SPRING] Spring simples por {duration:.0f}s")
        print("  Gire a roda e solte — deve voltar ao centro automaticamente.")
        t0 = time.time()
        while time.time() - t0 < duration:
            fb = self.get_feedback()
            if fb:
                # torque proporcional à posição (spring)
                torque = int(-stiffness * fb['enc_pos'] / (ENCODER_CPR / 360.0))
                torque = max(-1000, min(1000, torque))
            else:
                torque = 0
            self.send(torque)
            time.sleep(0.005)   # 200 Hz
        self.send(0)
        print("  Spring desligado.")
        self.print_stats()

    def test_sine_torque(self, amplitude=300, freq_hz=0.5, duration=10.0):
        """Aplica torque senoidal para verificar resposta do motor."""
        print(f"\n[SENO] Torque senoidal {amplitude}*sin({freq_hz}Hz) por {duration:.0f}s")
        t0 = time.time()
        while True:
            elapsed = time.time() - t0
            if elapsed >= duration:
                break
            torque = int(amplitude * math.sin(2 * math.pi * freq_hz * elapsed))
            self.send(torque)
            fb = self.get_feedback()
            if fb:
                print(f"\r  t={elapsed:.2f}s  torque={torque:+5d}  "
                      f"enc={fb['enc_pos']:+6d}cnt  RPM={fb['speedR']:+4d}",
                      end='', flush=True)
            time.sleep(0.005)
        print()
        self.send(0)
        print("  Seno concluído.")
        self.print_stats()

    def test_interactive(self):
        """Controle manual: digita torque [-1000, +1000]."""
        print("\n[INTERATIVO] Digite torque [-1000, +1000] ou 'q' para sair")
        print("  Exemplos: 200  -500  0")
        while True:
            fb = self.get_feedback()
            pos = f"{fb['enc_pos']:+6d}cnt ({fb['enc_deg']:+6.1f}°)" if fb else "---"
            try:
                line = input(f"  enc={pos}  torque> ").strip()
            except (KeyboardInterrupt, EOFError):
                break
            if line.lower() in ('q', 'quit', 'exit'):
                break
            try:
                torque = int(line)
                self.send(torque)
            except ValueError:
                print("  Erro: valor inteiro esperado")
        self.send(0)

    def test_encoder_info(self):
        """Imprime informações sobre o encoder e conversão de unidades."""
        print("\n[ENCODER INFO]")
        print(f"  MT6701 ABZ: PPR=1024, CPR={ENCODER_CPR}")
        print(f"  ROTATION_MAX_TICKS = {ROTATION_MAX}")
        print(f"  Lock-to-lock máx   = ±{ROTATION_MAX/ENCODER_CPR*360:.0f}° (±{ROTATION_MAX} counts)")
        print(f"  Resolução angular  = {360/ENCODER_CPR:.4f}°/count")
        print()
        print("  Lock-to-lock típicos:")
        for deg in [540, 720, 900, 1080]:
            counts = int(deg / 360 * ENCODER_CPR)
            ok = "✓" if counts <= ROTATION_MAX else "✗ EXCEDE MAX"
            print(f"    {deg:4d}° = ±{counts} counts  {ok}")
        print()
        print("  Nota: Feedback.speedL_meas = enc_pos (posição, não RPM!)")
        print("        Feedback.speedR_meas = RPM do motor")
        fb = self.get_feedback()
        if fb:
            print(f"\n  Posição atual: {fb['enc_pos']:+d} counts ({fb['enc_deg']:+.1f}°)")


# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description='Testa USART3 do STM32 ONE_AXIS_USART_VARIANT (500kbps)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Testes disponíveis:
  ping        Envia torque=0 e verifica feedback (padrão)
  sweep       Varre torque de -500 a +500 (motor gira levemente)
  spring      Simula spring de centralização (10s)
  sine        Torque senoidal para verificar resposta
  interactive Controle manual via terminal
  encoder     Mostra informações do encoder e posição atual
        """
    )
    parser.add_argument('--port', required=True, help='Porta serial (ex: /dev/ttyUSB0, COM4)')
    parser.add_argument('--baud', default=HOVER_BAUD, type=int,
                        help=f'Baud rate (padrão: {HOVER_BAUD})')
    parser.add_argument('--test',
                        choices=['ping', 'sweep', 'spring', 'sine', 'interactive', 'encoder'],
                        default='ping',
                        help='Teste a executar (padrão: ping)')
    args = parser.parse_args()

    if args.baud != HOVER_BAUD:
        print(f"⚠  Atenção: baud={args.baud} — o firmware ONE_AXIS_USART_VARIANT usa {HOVER_BAUD}!")
        print(f"   Se travar, verifique config.h #define USART3_BAUD {HOVER_BAUD}")

    print(f"Conectando a {args.port} @ {args.baud} baud...")
    tester = HoverTest(args.port, args.baud)
    tester.start()
    print("Conectado. Iniciando teste...\n")

    try:
        if   args.test == 'ping':        tester.test_ping()
        elif args.test == 'sweep':       tester.test_sweep_torque()
        elif args.test == 'spring':      tester.test_center_spring()
        elif args.test == 'sine':        tester.test_sine_torque()
        elif args.test == 'interactive': tester.test_interactive()
        elif args.test == 'encoder':
            # Precisa de alguns pacotes primeiro
            time.sleep(2)
            tester.test_encoder_info()
    except KeyboardInterrupt:
        print("\n  Interrompido pelo usuário.")
    finally:
        tester.stop()
        print(f"\n  Estatísticas finais: ", end='')
        tester.print_stats()


if __name__ == '__main__':
    main()
