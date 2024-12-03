# grupo 7
# M2 - subida (move up) P-mode
# H1 e H3 sentido horario - H2 e H4 sentido anti-horario
# rotacao alta nas quatro helices
# velocidade maxima de subida - 5 m/s P-mode
# A maior potencia é para o voo em P-mode, com F A = 1. Este fator pode ser ajustado durante a operaçao, nao sendo, necessariamente, constante.

import numpy as np
import skfuzzy as fuzzy
import skfuzzy.control as ctrl
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt
import time
import tkinter as tk
import math

mqttBroker = "test.mosquitto.org"
client = mqtt.Client(client_id="C213-Projeto2", clean_session=True)
client.connect(mqttBroker)

erro = ctrl.Antecedent(universe=np.arange(-120,120.1,0.1), label="erro")
erro['MN'] = fuzzy.trapmf(erro.universe, [-120, -120, -60, -20])  # Muito Negativo
erro['PN'] = fuzzy.trapmf(erro.universe, [-60, -20, -5, 0])   # Positivo Negativo
erro['ZE'] = fuzzy.trimf(erro.universe, [-5, 0, 5])      # Zero
erro['PP'] = fuzzy.trapmf(erro.universe, [0, 5, 20, 60])     # Positivo Positivo
erro['MP'] = fuzzy.trapmf(erro.universe, [20, 60, 120, 120])    # Muito Positivo
# erro.view()

var_erro = ctrl.Antecedent(universe=np.arange(-5,5.1,0.1), label="variação do erro")
var_erro['MN'] = fuzzy.trapmf(var_erro.universe, [-5, -5, -3, -1])    # Muito Negativo
var_erro['PN'] = fuzzy.trimf(var_erro.universe, [-3, -1, 0])     # Positivo Negativo
var_erro['ZE'] = fuzzy.trimf(var_erro.universe, [-1, 0, 1])      # Zero
var_erro['PP'] = fuzzy.trimf(var_erro.universe, [0, 1, 3])      # Positivo Positivo
var_erro['MP'] = fuzzy.trapmf(var_erro.universe, [1, 3, 5, 5])      # Muito Positivo
# var_erro.view()

potencia = ctrl.Consequent(universe=np.arange(0,1.05,0.05), label="potência do motor")
potencia['MB'] = fuzzy.trimf(potencia.universe, [0,0,0.25])
potencia['B'] = fuzzy.trimf(potencia.universe, [0,0.25,0.5])
potencia['M'] = fuzzy.trimf(potencia.universe, [0.25,0.5,0.75])
potencia['A'] = fuzzy.trimf(potencia.universe, [0.5,0.75,1])
potencia['MA'] = fuzzy.trimf(potencia.universe, [0.75,1,1])
potencia.view()

[plt.gca().lines[i].set_linewidth(2) for i in range(len(plt.gca().lines))]

fig = plt.gcf(); axes = fig.gca(); fig.set_size_inches(6, 2)
axes.set_xlabel(xlabel=f'{potencia.label} [%]'); axes.set_ylabel(ylabel='Pertinência $\mu_{V}$')
plt.title(f'Classificações para a variável {potencia.label}', fontweight='bold'); plt.legend(loc='upper right')
plt.show()

# Base de Regras
baseRegras = []
baseRegras.append(ctrl.Rule(erro['MN'] & var_erro['MN'], potencia['MB']))
baseRegras.append(ctrl.Rule(erro['MN'] & var_erro['PN'], potencia['B']))
baseRegras.append(ctrl.Rule(erro['MN'] & var_erro['ZE'], potencia['M']))
baseRegras.append(ctrl.Rule(erro['MN'] & var_erro['PP'], potencia['A']))
baseRegras.append(ctrl.Rule(erro['MN'] & var_erro['MP'], potencia['MA']))
baseRegras.append(ctrl.Rule(erro['PN'] & var_erro['MN'], potencia['MB']))
baseRegras.append(ctrl.Rule(erro['PN'] & var_erro['PN'], potencia['B']))
baseRegras.append(ctrl.Rule(erro['PN'] & var_erro['ZE'], potencia['M']))
baseRegras.append(ctrl.Rule(erro['PN'] & var_erro['PP'], potencia['A']))
baseRegras.append(ctrl.Rule(erro['PN'] & var_erro['MP'], potencia['A']))
baseRegras.append(ctrl.Rule(erro['ZE'] & var_erro['MN'], potencia['B']))
baseRegras.append(ctrl.Rule(erro['ZE'] & var_erro['PN'], potencia['B']))
baseRegras.append(ctrl.Rule(erro['ZE'] & var_erro['ZE'], potencia['M']))
baseRegras.append(ctrl.Rule(erro['ZE'] & var_erro['PP'], potencia['M']))
baseRegras.append(ctrl.Rule(erro['ZE'] & var_erro['MP'], potencia['A']))
baseRegras.append(ctrl.Rule(erro['PP'] & var_erro['MN'], potencia['B']))
baseRegras.append(ctrl.Rule(erro['PP'] & var_erro['PN'], potencia['M']))
baseRegras.append(ctrl.Rule(erro['PP'] & var_erro['ZE'], potencia['A']))
baseRegras.append(ctrl.Rule(erro['PP'] & var_erro['PP'], potencia['A']))
baseRegras.append(ctrl.Rule(erro['PP'] & var_erro['MP'], potencia['A']))
baseRegras.append(ctrl.Rule(erro['MP'] & var_erro['MN'], potencia['B']))
baseRegras.append(ctrl.Rule(erro['MP'] & var_erro['PN'], potencia['M']))
baseRegras.append(ctrl.Rule(erro['MP'] & var_erro['ZE'], potencia['A']))
baseRegras.append(ctrl.Rule(erro['MP'] & var_erro['PP'], potencia['MA']))
baseRegras.append(ctrl.Rule(erro['MP'] & var_erro['MP'], potencia['MA']))

controlePosicao = ctrl.ControlSystemSimulation(ctrl.ControlSystem(baseRegras))

def ControleDrone(setPoint, origem):
    # setPoint = 110
    fator_ajuste = 0.96
    Umax = 5
    posicaoAtual, posicao = [origem, [origem]]
    erroAnterior = setPoint-posicaoAtual

    if erroAnterior < 30:
        fator_ajuste = 0.86
    if erroAnterior < 15:
        fator_ajuste = 0.75
    if erroAnterior < 10:
        fator_ajuste = 0.7
    if erroAnterior >= 30:
        fator_ajuste = 0.92
    if erroAnterior >= 60:
        fator_ajuste = 0.94
    if erroAnterior >= 90:
        fator_ajuste = 0.96

    tempo = np.arange(0, 100, 1)
    for _ in range(1, np.max(tempo)+1):
        erroAtual = setPoint-posicaoAtual
        controlePosicao.input[erro.label] = erroAtual
        
        delta_erroAtual = erroAnterior - erroAtual
        controlePosicao.input[var_erro.label] = delta_erroAtual
        
        controlePosicao.compute()
        potencia_H13 = controlePosicao.output[potencia.label]
        potencia_H24 = potencia_H13

        posicaoAtual = fator_ajuste * posicaoAtual * 1.01398 + 0.5*(Umax*potencia_H13 + Umax*potencia_H24)
        posicao.append(posicaoAtual)
        erroAnterior = erroAtual
        client.publish("Drone/Posicao", posicaoAtual)
        client.publish("Drone/Erro", erroAtual)
        client.publish("Drone/VarErro", delta_erroAtual)
        client.publish("Drone/Potencia", potencia_H13)
        time.sleep(0.1)
        
    plt.plot(tempo, posicao)
    plt.axhline(y=setPoint, color="red", linestyle="--", label=f"Setpoint ({setPoint} m)")
    plt.title("Posição do Drone ao Longo do Tempo")
    plt.xlabel("Tempo [s]")
    plt.ylabel("Posição [m]")
    plt.grid(True)
    plt.show()

class DroneController:
    def __init__(self, root):
        self.root = root
        self.root.title("Controle do Drone")
        self.root.geometry("600x400")
        self.root.configure(bg="#e6e6e6")  # Fundo cinza claro

        # Variáveis de controle
        self.altitude = tk.DoubleVar(value=0)
        self.direction_x = tk.DoubleVar(value=0)
        self.direction_y = tk.DoubleVar(value=0)
        self.set_point = tk.DoubleVar(value=0)
        self.origin = tk.StringVar(value="")

        # Joystick Esquerdo
        self.left_joystick = tk.Frame(self.root, bg="#e6e6e6")
        self.left_joystick.place(x=50, y=100)

        self.arrow_up_left = tk.Button(
            self.left_joystick, text="▲", font=("Arial", 12), width=3, height=2, bg="#cccccc", fg="black",
            command=lambda:ControleDrone(int(self.set_point.get()), int(self.origin.get())))
        self.arrow_up_left.grid(row=0, column=1, pady=5)

        self.arrow_left_left = tk.Button(
            self.left_joystick, text="◄", font=("Arial", 12), width=3, height=2, bg="#cccccc", fg="black",
            command=lambda: self.move("left_left"))
        self.arrow_left_left.grid(row=1, column=0, padx=5)

        self.space_center_left = tk.Label(self.left_joystick, bg="#e6e6e6", width=3, height=2)
        self.space_center_left.grid(row=1, column=1)

        self.arrow_right_left = tk.Button(
            self.left_joystick, text="►", font=("Arial", 12), width=3, height=2, bg="#cccccc", fg="black",
            command=lambda: self.move("right_left"))
        self.arrow_right_left.grid(row=1, column=2, padx=5)

        self.arrow_down_left = tk.Button(
            self.left_joystick, text="▼", font=("Arial", 12), width=3, height=2, bg="#cccccc", fg="black",
            command=lambda: self.move("down_left"))
        self.arrow_down_left.grid(row=2, column=1, pady=5)

        # Joystick Direito
        self.right_joystick = tk.Frame(self.root, bg="#e6e6e6")
        self.right_joystick.place(x=450, y=100)

        self.arrow_up_right = tk.Button(
            self.right_joystick, text="▲", font=("Arial", 12), width=3, height=2, bg="#cccccc", fg="black",
            command=lambda: self.move("up_right"))
        self.arrow_up_right.grid(row=0, column=1, pady=5)

        self.arrow_left_right = tk.Button(
            self.right_joystick, text="◄", font=("Arial", 12), width=3, height=2, bg="#cccccc", fg="black",
            command=lambda: self.move("left_right"))
        self.arrow_left_right.grid(row=1, column=0, padx=5)

        self.space_center_right = tk.Label(self.right_joystick, bg="#e6e6e6", width=3, height=2)
        self.space_center_right.grid(row=1, column=1)

        self.arrow_right_right = tk.Button(
            self.right_joystick, text="►", font=("Arial", 12), width=3, height=2, bg="#cccccc", fg="black",
            command=lambda: self.move("right_right"))
        self.arrow_right_right.grid(row=1, column=2, padx=5)

        self.arrow_down_right = tk.Button(
            self.right_joystick, text="▼", font=("Arial", 12), width=3, height=2, bg="#cccccc", fg="black",
            command=lambda: self.move("down_right"))
        self.arrow_down_right.grid(row=2, column=1, pady=5)

        # Set Point e Origem (Centralizados)
        self.set_point_frame = tk.Frame(self.root, bg="#e6e6e6")
        self.set_point_frame.place(relx=0.5, rely=0.5, anchor="center")

        tk.Label(self.set_point_frame, text="Set Point", font=("Arial", 12, "bold"), bg="#e6e6e6", fg="black").grid(row=0, column=0, pady=5)
        self.set_point_entry = tk.Entry(self.set_point_frame, textvariable=self.set_point, width=10, font=("Arial", 10))
        self.set_point_entry.grid(row=0, column=1, pady=5, padx=10)

        tk.Label(self.set_point_frame, text="Origem", font=("Arial", 12, "bold"), bg="#e6e6e6", fg="black").grid(row=1, column=0, pady=5)
        self.origin_entry = tk.Entry(self.set_point_frame, textvariable=self.origin, width=10, font=("Arial", 10))
        self.origin_entry.grid(row=1, column=1, pady=5, padx=10)

        # Botão Return to Home
        self.home_button = tk.Button(
            self.root, text="Return to Home", font=("Arial", 10), bg="#999999", fg="white", command=self.return_to_home)
        self.home_button.place(relx=0.5, rely=0.85, anchor="center", width=120, height=40)

    def move(self, direction):
        """Movimenta o drone na direção especificada."""
        if direction.startswith("up"):
            self.direction_y.set(self.direction_y.get() + 1)
        elif direction.startswith("down"):
            self.direction_y.set(self.direction_y.get() - 1)
        elif direction.startswith("left"):
            self.direction_x.set(self.direction_x.get() - 1)
        elif direction.startswith("right"):
            self.direction_x.set(self.direction_x.get() + 1)
        # print(f"Movimento ({direction}): X={self.direction_x.get()}, Y={self.direction_y.get()}")

    def return_to_home(self):
        """Reseta as posições para retornar ao ponto inicial."""
        # print("Drone retornando para o ponto inicial...")
        self.altitude.set(0)
        self.direction_x.set(0)
        self.direction_y.set(0)
        self.set_point.set(0)
        self.origin.set("")

    def confirm_values(self):
        """Confirma os valores inseridos em Set Point e Origem."""
        # print(f"Set Point: {self.set_point.get()} | Origem: {self.origin.get()}")


root = tk.Tk()
app = DroneController(root)
root.mainloop()