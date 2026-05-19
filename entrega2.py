import itertools
from itertools import combinations
from sys import implementation

def build_camp(camp_size, habs, generators, labs, deposits, airlocks, craters):
    variables = []
    global CRATERS
    CRATERS = craters
    global SIZE
    SIZE = camp_size

    for _ in range(habs): variables.append("hab")
    for _ in range(generators): variables.append("gen")
    for _ in range(labs): variables.append("lab")
    for _ in range(deposits): variables.append("dep")
    for _ in range(airlocks): variables.append("air")

    dominios = []

    celdas_borde = []
    for col in range(camp_size[1]):
        if (0, col) not in craters:
            celdas_borde.append((0, col))
        if (camp_size[0] -1, col) not in craters:
            celdas_borde.append((camp_size[0] - 1, col))
    for fil in range(camp_size[0]):
        if (fil, 0) not in craters:
            celdas_borde.append((fil, 0))
        if (fil, camp_size[1] - 1) not in craters:
            celdas_borde.append((fil, camp_size[1] - 1))

    celdas_internas = []
    for fil in range(camp_size[0] - 1):
        for col in range(camp_size[1] - 1):
            #Pregunta si no es borde
            if fil != 0 and col != 0:
                if (fil, col) not in craters:
                    celdas_internas.append((fil, col))

    celdas_utiles = celdas_internas + celdas_borde

    dominios["hab"] = celdas_internas
    dominios["air"] = celdas_borde

    for tipo in ("gen", "lab", "dep"):
        dominios[tipo] = celdas_utiles

    restricciones = []
    for mod1, mod2 in itertools.combinations(variables, 2):
        restricciones.append((mod1, mod2), diferentes)
        restricciones.append((mod1, mod2), adyacencia_generador_habitacion)
        restricciones.append((mod1, mod2), adyacencia_generadores)
    restricciones.append(variables, adyacente_libre)




    return None


#R1: solo un modulo por celda
def diferentes(variables, values):
    return values[0] != values[1]

#R2: crateres ya planteados en dominios
#R3: esclusas ya planteadas en dominios
#R4: habitaciones ya planteadas en dominios

def adyacente(elem1, elem2):
    return abs(elem1[0] - elem2[0]) + abs(elem1[1] - elem2[1]) == 1

#R5: generador no puede ser adyacente a una habitacion
def adyacencia_generador_habitacion(variables, values):
    if "hab" in variables and "gen" in variables:
        return not adyacente(values[0], values[1])

#R6: generadores no adyacentes
def adyacencia_generadores(variables, values):
    if variables.count("gen") == 2:
        return not adyacente(values[0], values[1])

#R7: laboratorio es adyacente a un deposito
def adyacencia_laboratorio_deposito(variables, values):
    if "lab" in variables and "dep" in variables:
        return adyacente(values[0], values[1])

#R8: debe haber una celda libre adyacente a una habitacion
def adyacente_libre(variables, values):
    posibilidades = [
        (0, -1),
        (0, 1),
        (-1, 0),
        (1, 0),
    ]

    for i, variable in enumerate(variables):
        if "hab" == variable:
            contador = 0
            for movimiento in posibilidades:
                nuevo_movimiento = values[i] + movimiento
                if 0 <= nuevo_movimiento[0] < SIZE[0] and 0 <= nuevo_movimiento[1] < SIZE[1]:
                    if nuevo_movimiento not in CRATERS and nuevo_movimiento not in values:
                        contador += 1

            if

    return False
