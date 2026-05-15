from simpleai.search import (
    SearchProblem,
    breadth_first,
    depth_first,
    limited_depth_first,
    uniform_cost,
    iterative_limited_depth_first, astar,
)

MOVIMIENTOS = ((0,-1), (-1,0), (0,1), (1,0))
SOBREMARCHAS = ((0,-2), (-2,0), (0,2), (2,0))

# todas las coordenadas son en formato (fila, columna)


class Entrega1(SearchProblem):
    def __init__ (self, rover_inicio, bateria_inicial, zonas_sombra, muestras_igneas, muestras_sedimentarias):

        self.zonas_sombra = [(0, 1), (0, 2)],
        taladro = None
        cargas = 0

        inicial = (rover_inicio, bateria_inicial, taladro, cargas, muestras_igneas, muestras_sedimentarias)
        super(Entrega1, self).__init__(inicial)

    def actions(self, state):
        available_actions = []

        posicion_rover, bateria, taladro, cargas, muestras_igneas, muestras_sedimentarias = state


        if bateria >= 1:
            # Movimientos simples
            for movimiento in MOVIMIENTOS:
                nueva_posicion = (posicion_rover[0] + movimiento[0], posicion_rover[1] + movimiento[1])
                available_actions.append(("moverse", nueva_posicion))

            # Equipar taladro
            if posicion_rover in muestras_igneas:
                if taladro != "termico":
                    available_actions.append(("equipar", "termico"))
            if posicion_rover in muestras_sedimentarias:
                if taladro != "percusión":
                    available_actions.append(("equipar", "percusión"))

            # Depositar cargas
            if cargas == 2:
                available_actions.append(("depositar", None))

            if (len(muestras_igneas) + len(muestras_sedimentarias)) == 0 and cargas == 1:
                available_actions.append(("depositar", None))

            # Desplegar paneles solares
            if posicion_rover not in self.zonas_sombra and bateria < 20:
                available_actions.append(("recargar", None))

        if bateria >= 3:
            # Recolectar muestra
            if cargas < 2:
                if posicion_rover in muestras_igneas:
                    if taladro == "termico":
                        available_actions.append(("recolectar", "ignea"))
                if posicion_rover in muestras_sedimentarias:
                    if taladro == "percusión":
                        available_actions.append(("recolectar", "sedimentaria"))

        if bateria >= 4:
            #Movimientos dobles
            for sobremarcha in SOBREMARCHAS:
                nueva_posicion = (posicion_rover[0] + sobremarcha[0], posicion_rover[1] + sobremarcha[1])
                available_actions.append(("sobremarcha", nueva_posicion))


        return available_actions

    def is_goal(self, state):
        posicion_rover, bateria, taladro, cargas, muestras_igneas, muestras_sedimentarias = state
        return (len(muestras_igneas) + len(muestras_sedimentarias)) == 0 and bateria > 0 and cargas == 0

    def result(self, state, action):
        estado = list(state)
        accion, descripcion = action

        if accion == "moverse":
            estado[0] = descripcion
            estado[1] -= 1
        elif accion == "sobremarcha":
            estado[0] = descripcion
            estado[1] -= 4
        elif accion == "equipar":
            estado[2] = descripcion
            estado[1] -= 1
        elif accion == "recolectar":
            if descripcion == "ignea":
                igneas = list(estado[4])
                igneas.remove(estado[0])
                estado[4] = tuple(igneas)
            else:
                sedimentarias = list(estado[5])
                sedimentarias.remove(estado[0])
                estado[5] = tuple(sedimentarias)
            estado[1] -= 3
        elif accion == "depositar":
            estado[3] = 0
            estado[1] -= 1
        elif accion == "recargar":
            if estado[1] > 10:
                estado[1] = 20
            else:
                estado[1] += 10

        return tuple(estado)

    def cost(self, state1, action, state2):
        posicion_rover, bateria, taladro, cargas, muestras_igneas, muestras_sedimentarias = state2
        accion, descripcion = action

        if accion == "moverse":
            return 1
        elif accion == "sobremarcha":
            return 1
        elif accion == "equipar":
            return 3
        elif accion == "recolectar":
            return 2
        elif accion == "depositar":
            return cargas
        elif accion == "recargar":
            return 4

    def heuristic(self,state):
        posicion_rover, bateria, taladro, cargas, muestras_igneas, muestras_sedimentarias = state
        muestras = muestras_igneas + muestras_sedimentarias
        distancias_manhattan = []
        max_distancia = 0
        cambio_taladro = 0
        for muestra in muestras:
            distancia = abs(posicion_rover[0] - muestra[0]) + abs(posicion_rover[1] - muestra[1])
            distancias_manhattan.append(distancia)
        if len(distancias_manhattan) != 0:
            max_distancia = max(distancias_manhattan)
        if len(muestras_igneas) == 0 and taladro == "termico":
            cambio_taladro = 3
        elif len(muestras_sedimentarias) == 0 and taladro == "percusión":
            cambio_taladro = 3
        return cargas + len(muestras) * 2 + max_distancia / 2 + cambio_taladro

def planear_rover(rover_inicio, bateria_inicial, zonas_sombra, muestras_igneas, muestras_sedimentarias):
    problem = Entrega1(rover_inicio, bateria_inicial, zonas_sombra, muestras_igneas, muestras_sedimentarias)
    result = astar(problem, graph_search=True).path()
    return [accion[0]for accion in result]


def main():
    result = planear_rover((0, 0), 20, ((0, 1), (0, 2)), ((1, 1), (1, 2)), ((2, 3), ))
    for accion in result:
        print(accion)

if __name__ == '__main__':
    main()
