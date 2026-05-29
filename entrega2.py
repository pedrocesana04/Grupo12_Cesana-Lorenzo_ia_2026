import itertools

from simpleai.search import CspProblem, backtrack, min_conflicts

def build_camp(camp_size, habs, generators, labs, deposits, airlocks, craters):
    variables = []
    global CRATERS
    CRATERS = craters
    global SIZE
    SIZE = camp_size

    dominios = {}

    #Definimos las celdas del borde de la cuadrilla.
    celdas_borde = []
    for col in range(camp_size[1]):
        if (0, col) not in craters:
            celdas_borde.append((0, col))
        if (camp_size[0] -1, col) not in craters:
            celdas_borde.append((camp_size[0] - 1, col))
    for fil in range(camp_size[0]):
        #Pregunta si no esta dentro de los cráteres.
        if (fil, 0) not in craters:
            celdas_borde.append((fil, 0))
        if (fil, camp_size[1] - 1) not in craters:
            celdas_borde.append((fil, camp_size[1] - 1))

    #Definimos las celdas internas de la cuadrilla.
    celdas_internas = []
    for fil in range(camp_size[0] - 1):
        for col in range(camp_size[1] - 1):
            #Pregunta si no es borde
            if fil != 0 and col != 0:
                #Pregunta si no esta dentro de los cráteres.
                if (fil, col) not in craters:
                    celdas_internas.append((fil, col))

    #Las celdas utiles serán las que no contengan cráteres.
    celdas_utiles = celdas_internas + celdas_borde

    #Definimos las variables y sus dominios.
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

    #Cargamos las restricciones definidas en una lista de restricciones.
    restricciones = []
    depositos = [dep for dep in variables if dep.startswith("dep")]
    #Verificamos restricciones para todas las combinaciones posibles de variables.
    for mod1, mod2 in itertools.combinations(variables, 2):
        restricciones.append(((mod1, mod2), diferentes))
        restricciones.append(((mod1, mod2), adyacencia_generador_habitacion))
        restricciones.append(((mod1, mod2), adyacencia_generadores))
    #Chequeamos la condición exclusiva de habitaciones para cada habitación.
    for hab in variables:
        if hab.startswith("hab"):
            tupla = (hab,) + tuple(variables)
            restricciones.append((tupla, adyacente_libre))
    # Chequeamos la condición exclusiva de habitaciones para cada laboratorio.
    for lab in variables:
        if lab.startswith("lab"):
            tupla = (lab,) + tuple(depositos)
            restricciones.append((tupla, adyacencia_laboratorio_deposito))

    #Llamos a SimpleAi para resolver el problema
    problem = CspProblem(variables, dominios, restricciones)
    result = backtrack(problem)

    #Verificamos que haya una solución.
    if result is None: return None

    #Cargamos el resultado en una lista en el formato deseado de presentación.
    resultado = []
    for var, coords in result.items():
        tipo = var[:3]
        resultado.append((tipo, coords[0], coords[1]))

    #Devolvemos la lista con el resultado.
    return resultado


#R1: solo debe haber un modulo por celda.
def diferentes(variables, values):
    return values[0] != values[1]

#R2: cráteres ya planteados en dominios.
#R3: esclusas ya planteadas en dominios.
#R4: habitaciones ya planteadas en dominios.

#Definimos una función para comprobar la adyacencia de dos módulos.
def adyacente(elem1, elem2):
    return abs(elem1[0] - elem2[0]) + abs(elem1[1] - elem2[1]) == 1

#R5: Un generador no puede ser adyacente a una habitacion.
def adyacencia_generador_habitacion(variables, values):
    if any(v.startswith("hab") for v in variables) and any(v.startswith("gen") for v in variables):
        return not adyacente(values[0], values[1])
    return True

#R6: Los generadores no pueden ser adyacentes entre si.
def adyacencia_generadores(variables, values):
    if sum(v.startswith("gen") for v in variables) == 2:
        return not adyacente(values[0], values[1])
    return True

#R7: Un laboratorio debe ser adyacente a un deposito.
def adyacencia_laboratorio_deposito(variables, values):
    laboratorio = values[0]
    depositos = values[1:]

    return adyacente(values[0], values[1])

#R8: Cada habitación debe tener por lo menos una celda adyacente libre.
def adyacente_libre(variables, values):
    posibilidades = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    #Verificamos adyacencia por cada movimiento.
    for movimiento in posibilidades:
        nx = values[0][0] + movimiento[0]
        ny = values[0][1] + movimiento[1]
        nuevo_movimiento = (nx, ny)
        if 0 <= nuevo_movimiento[0] < SIZE[0] and 0 <= nuevo_movimiento[1] < SIZE[1]:
            if nuevo_movimiento not in CRATERS and nuevo_movimiento not in values:
                return True
    return False

#Para probar.
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
