from simpleai.search import SearchProblem, astar


class RoverProblem(SearchProblem):
    def __init__(self, rover_inicio, bateria_inicial, zonas_sombra, muestras_igneas, muestras_sedimentarias):
        # Convertimos las listas a tuplas ordenadas para que los estados sean "hashables"
        self.zonas_sombra = set(zonas_sombra)  # Set para búsquedas O(1)

        estado_inicial = (
            rover_inicio,
            bateria_inicial,
            None,  # Taladro activo inicialmente
            0,  # Carga inicial (cantidad)
            tuple(sorted(muestras_igneas)),
            tuple(sorted(muestras_sedimentarias))
        )
        super().__init__(initial_state=estado_inicial)

    def actions(self, state):
        pos, bat, drill, cargo, i_left, s_left = state
        r, c = pos
        acciones = []

        # 1. Moverse: Toma 1 min, consume 1 bat (Batería no puede llegar a 0 -> bat > 1)
        if bat > 1:
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if nr >= 0 and nc >= 0:  # Evitamos salirnos de coordenadas válidas
                    acciones.append(("moverse", (nr, nc)))

        # 2. Sobremarcha: Toma 1 min, consume 4 bat -> bat > 4
        if bat > 4:
            for dr, dc in [(-2, 0), (2, 0), (0, -2), (0, 2)]:
                nr, nc = r + dr, c + dc
                if nr >= 0 and nc >= 0:
                    acciones.append(("sobremarcha", (nr, nc)))

        # 3. Equipar: Toma 3 min, consume 1 bat -> bat > 1
        if bat > 1:
            if drill != "termico":
                acciones.append(("equipar", "termico"))
            if drill != "percusion":
                acciones.append(("equipar", "percusion"))

        # 4. Perforar y Recolectar: Toma 2 min, consume 3 bat -> bat > 3
        # Requiere espacio en carga (< 2)
        if bat > 3 and cargo < 2:
            if drill == "termico" and pos in i_left:
                acciones.append(("recolectar", "ignea"))
            if drill == "percusion" and pos in s_left:
                acciones.append(("recolectar", "sedimentaria"))

        # 5. Depositar (Entregar): Toma 1 min por muestra, consume 1 bat -> bat > 1
        # Requiere carga. Se puede si carga == 2, o si es la última muestra global posible
        if bat > 1 and cargo > 0:
            muestras_globales_restantes = len(i_left) + len(s_left)
            if cargo == 2 or muestras_globales_restantes == 0:
                acciones.append(("depositar", None))

        # 6. Recargar: Toma 4 min, suma 10 bat (Max 20). No gasta para iniciar
        # Restricción: No permitida en zona de sombra. Solo vale la pena si la batería es < 20
        if pos not in self.zonas_sombra and bat < 20:
            acciones.append(("recargar", None))

        return acciones

    def result(self, state, action):
        pos, bat, drill, cargo, i_left, s_left = state
        tipo_accion, param = action

        if tipo_accion == "moverse":
            return (param, bat - 1, drill, cargo, i_left, s_left)

        elif tipo_accion == "sobremarcha":
            return (param, bat - 4, drill, cargo, i_left, s_left)

        elif tipo_accion == "equipar":
            return (pos, bat - 1, param, cargo, i_left, s_left)

        elif tipo_accion == "recolectar":
            # Eliminamos de la tupla la muestra recolectada
            nuevo_cargo = cargo + 1
            if param == "ignea":
                nuevo_i = tuple(x for x in i_left if x != pos)
                return (pos, bat - 3, drill, nuevo_cargo, nuevo_i, s_left)
            else:  # sedimentaria
                nuevo_s = tuple(x for x in s_left if x != pos)
                return (pos, bat - 3, drill, nuevo_cargo, i_left, nuevo_s)

        elif tipo_accion == "depositar":
            return (pos, bat - 1, drill, 0, i_left, s_left)

        elif tipo_accion == "recargar":
            nueva_bat = min(20, bat + 10)
            return (pos, nueva_bat, drill, cargo, i_left, s_left)

    def cost(self, state1, action, state2):
        tipo_accion, _ = action

        if tipo_accion in ("moverse", "sobremarcha"):
            return 1
        elif tipo_accion == "equipar":
            return 3
        elif tipo_accion == "recolectar":
            return 2
        elif tipo_accion == "depositar":
            # El estado 1 (antes de depositar) guarda la cantidad de carga. 1 min por muestra.
            _, _, _, cargo, _, _ = state1
            return 1 * cargo
        elif tipo_accion == "recargar":
            return 4

        return 0

    def is_goal(self, state):
        _, _, _, cargo, i_left, s_left = state
        # El objetivo es que no queden muestras en el mapa ni en la bahía de carga
        return len(i_left) == 0 and len(s_left) == 0 and cargo == 0

    def heuristic(self, state):
        pos, bat, drill, cargo, i_left, s_left = state
        h = 0
        muestras_restantes_mapa = len(i_left) + len(s_left)

        # 1. Costo ineludible de recolección (2 min c/u)
        h += 2 * muestras_restantes_mapa

        # 2. Costo ineludible de depositar todo (mapa + carga actual) (1 min c/u)
        h += 1 * (muestras_restantes_mapa + cargo)

        # 3. Costos ineludibles por cambio de taladros
        if len(i_left) > 0 and len(s_left) > 0:
            # Hay de ambos tipos. Se van a requerir los dos.
            if drill is None:
                h += 6  # Necesita equipar uno y luego el otro
            else:
                h += 3  # Ya tiene uno, solo le falta equipar el otro al menos una vez
        elif len(i_left) > 0:
            if drill != "termico": h += 3
        elif len(s_left) > 0:
            if drill != "percusion": h += 3

        # 4. Costo de desplazamiento (Distancia Manhattan a la muestra más cercana)
        if muestras_restantes_mapa > 0:
            todas_las_muestras = i_left + s_left
            distancias = [abs(pos[0] - m[0]) + abs(pos[1] - m[1]) for m in todas_las_muestras]
            min_distancia = min(distancias)

            # Lo dividimos por 2 considerando el mejor caso posible (usando todo sobremarcha)
            h += min_distancia // 2

        return h


def planear_rover(rover_inicio, bateria_inicial, zonas_sombra, muestras_igneas, muestras_sedimentarias):
    """
    Api requerida para ejecutar la búsqueda.
    """
    problema = RoverProblem(
        rover_inicio,
        bateria_inicial,
        zonas_sombra,
        muestras_igneas,
        muestras_sedimentarias
    )

    # Graph search optimiza estados visitados previamente (esencial al haber ciclos)
    resultado = astar(problema, graph_search=True)

    acciones_finales = []
    if resultado is not None:
        # Extraemos solo las acciones del path devuelto (ignorando el estado raíz/None)
        for accion, _ in resultado.path()[1:]:
            acciones_finales.append(accion)

    return acciones_finales

def main():
    result = planear_rover((0, 0), 20, ((0, 1), (0, 2)), ((1, 1), (1, 2)), ((2, 3), ))
    for accion in result:
        print(accion)

if __name__ == '__main__':
    main()