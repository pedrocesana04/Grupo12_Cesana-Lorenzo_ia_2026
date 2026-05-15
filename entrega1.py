from simpleai.search import (
    SearchProblem,
    breadth_first,
    depth_first,
    limited_depth_first,
    uniform_cost,
    iterative_limited_depth_first, astar,
)

N = 4
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
                if 0 <= nueva_posicion[0] < N and 0 <= nueva_posicion[1] < N:
                    available_actions.append(("moverse", nueva_posicion))

            # Equipar taladro
            if taladro is None:
                available_actions.append(("equipar", "termico"))
                available_actions.append(("equipar", "percusión"))
            else:
                if taladro == "termico":
                    available_actions.append(("equipar", "percusión"))
                else:
                    available_actions.append(("equipar", "termico"))

            # Depositar cargas
            if cargas == 2 or (len(muestras_igneas) + len(muestras_sedimentarias)) == 1:
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
                if 0 <= nueva_posicion[0] < N and 0 <= nueva_posicion[1] < N:
                    available_actions.append(("sobremarcha", nueva_posicion))


        return available_actions

    def is_goal(self, state):
        posicion_rover, bateria, taladro, cargas, muestras_igneas, muestras_sedimentarias = state
        return (len(muestras_igneas) + len(muestras_sedimentarias)) == 0 and bateria < 0 and cargas == 0

    def result(self, state, action):
        posicion_rover, bateria, taladro, cargas, muestras_igneas, muestras_sedimentarias = state
        accion, descripcion = action

        if accion == "moverse":
            posicion_rover = descripcion
            bateria -= 1
        elif accion == "sobremarcha":
            posicion_rover = descripcion
            bateria -= 4
        elif accion == "equipar":
            taladro = descripcion
            bateria -= 1
        elif accion == "recolectar":
            if descripcion == "ignea":
                igneas = list(muestras_igneas)
                igneas.remove(posicion_rover)
                muestras_igneas = tuple(igneas)
            else:
                sedimentarias = list(muestras_sedimentarias)
                sedimentarias.remove(posicion_rover)
                muestras_sedimentarias = tuple(sedimentarias)
            bateria -= 3
        elif accion == "depositar":
            cargas = 0
            bateria -= 1
        elif accion == "recargar":
            if bateria > 10:
                bateria = 20
            else:
                bateria += 10

        return (posicion_rover, bateria, taladro, cargas, muestras_igneas, muestras_sedimentarias)

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
            if cargas == 2:
                return 2
            else:
                return 1
        elif accion == "recargar":
            return 4

    def heuristic(self,state):
        posicion_rover, bateria, taladro, cargas, muestras_igneas, muestras_sedimentarias = state
        muestras = muestras_igneas + muestras_sedimentarias
        distancias_manhattan = []
        max_distancia = 0
        for muestra in muestras:
            distancia = abs(posicion_rover[0] - muestra[0]) + abs(posicion_rover[1] - muestra[1])
            distancias_manhattan.append(distancia)
        if len(distancias_manhattan) == 0:
            max_distancia = max(distancias_manhattan)
        return cargas + len(muestras) * 2 + max_distancia/2 + 3

def planear_rover(rover_inicio, bateria_inicial, zonas_sombra, muestras_igneas, muestras_sedimentarias):
    problem = Entrega1(rover_inicio, bateria_inicial, zonas_sombra, muestras_igneas, muestras_sedimentarias)
    return astar(problem)


def main():
    result = planear_rover((0, 0), 20, ((0, 1), (0, 2)), ((1, 1), (1, 2)), ((2, 3), ))
    for accion in result.path():
        print(accion)

if __name__ == '__main__':
    main()
