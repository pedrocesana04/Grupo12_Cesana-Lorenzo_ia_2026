import itertools
from dis import deoptmap

from simpleai.search import CspProblem, backtrack, min_conflicts


def build_camp(camp_size, habs, generators, labs, deposits, airlocks, craters):
    variables = []
    global CRATERS
    CRATERS = craters
    global SIZE
    SIZE = camp_size

    dominios = {}

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

    for i in range(habs):
        variables.append(f"hab{i}")
        dominios[f"hab{i}"] = celdas_internas
    for i in range(generators):
        variables.append(f"gen{i}")
        dominios[f"gen{i}"] = celdas_utiles
    for i in range(labs):
        variables.append(f"lab{i}")
        dominios[f"lab{i}"] = celdas_utiles
    for i in range(deposits):
        variables.append(f"dep{i}")
        dominios[f"dep{i}"] = celdas_utiles
    for i in range(airlocks):
        variables.append(f"air{i}")
        dominios[f"air{i}"] = celdas_borde

    restricciones = []
    depositos = [dep for dep in variables if dep.startswith("dep")]
    for mod1, mod2 in itertools.combinations(variables, 2):
        restricciones.append(((mod1, mod2), diferentes))
        restricciones.append(((mod1, mod2), adyacencia_generador_habitacion))
        restricciones.append(((mod1, mod2), adyacencia_generadores))
    for hab in variables:
        if hab.startswith("hab"):
            tupla = (hab,) + tuple(variables)
            restricciones.append((tupla, adyacente_libre))
    for lab in variables:
        if lab.startswith("lab"):
            tupla = (lab,) + tuple(depositos)
            restricciones.append((tupla, adyacencia_laboratorio_deposito))


    problem = CspProblem(variables, dominios, restricciones)
    result = backtrack(problem)

    if result is None: return None

    resultado = []
    for var, coords in result.items():
        tipo = var[:3]
        resultado.append((tipo, coords[0], coords[1]))

    return resultado


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
    if any(v.startswith("hab") for v in variables) and any(v.startswith("gen") for v in variables):
        return not adyacente(values[0], values[1])
    return True

#R6: generadores no adyacentes
def adyacencia_generadores(variables, values):
    if sum(v.startswith("gen") for v in variables) == 2:
        return not adyacente(values[0], values[1])
    return True

#R7: laboratorio es adyacente a un deposito VER
def adyacencia_laboratorio_deposito(variables, values):
    laboratorio = values[0]
    depositos = values[1:]

    #ver!

    if any(v.startswith("lab") for v in variables) and any(v.startswith("dep") for v in variables):
        return adyacente(values[0], values[1])
    return False

#R8: debe haber una celda libre adyacente a una habitacion
def adyacente_libre(variables, values):
    posibilidades = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    for movimiento in posibilidades:
        nx = values[0][0] + movimiento[0]
        ny = values[0][1] + movimiento[1]
        nuevo_movimiento = (nx, ny)
        if 0 <= nuevo_movimiento[0] < SIZE[0] and 0 <= nuevo_movimiento[1] < SIZE[1]:
            if nuevo_movimiento not in CRATERS and nuevo_movimiento not in values:
                return True
    return False

if __name__ == "__main__":
    resultado = build_camp(
        camp_size=(5, 6),
        habs=2,
        generators=1,
        labs=1,
        deposits=2,
        airlocks=1,
        craters=[(2, 2), (2, 3)],
    )

    for tupla in resultado:
        print(tupla)
