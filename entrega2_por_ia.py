from simpleai.search import CspProblem, backtrack, MOST_CONSTRAINED_VARIABLE


def build_camp(camp_size, habs, generators, labs, deposits, airlocks, craters):
    """
    Genera la distribución del campamento base marciano utilizando un CSP.
    Retorna una lista de tuplas (tipo, fila, columna) o None si no es posible.
    """
    R, C = camp_size
    craters_set = set(craters)

    # 1. Definición de Variables
    counts = {
        "hab": habs,
        "gen": generators,
        "lab": labs,
        "dep": deposits,
        "air": airlocks
    }

    variables = []
    for m_type, count in counts.items():
        for i in range(count):
            variables.append(f"{m_type}_{i}")

    # Si no hay módulos a ubicar, la solución es un campamento vacío
    if not variables:
        return []

    # Caso imposible temprano: Laboratorios sin depósitos
    if labs > 0 and deposits == 0:
        return None

    # 2. Definición de Dominios
    domains = {}
    for v in variables:
        m_type = v.split('_')[0]
        valid_cells = []

        for r in range(R):
            for c in range(C):
                if (r, c) in craters_set:
                    continue  # Cráteres intransitables

                if m_type == "air":
                    # Esclusas solo en los bordes
                    if r == 0 or r == R - 1 or c == 0 or c == C - 1:
                        valid_cells.append((r, c))
                elif m_type == "hab":
                    # Habitacionales solo en el interior
                    if 0 < r < R - 1 and 0 < c < C - 1:
                        valid_cells.append((r, c))
                else:
                    valid_cells.append((r, c))

        # Si alguna variable se queda sin posiciones posibles, el problema no tiene solución
        if not valid_cells:
            return None

        domains[v] = valid_cells

    # 3. Restricciones
    constraints = []

    # Restricción: Sin superposición
    def all_diff(variables, values):
        return values[0] != values[1]

    # Restricción: No adyacencia (distancia Manhattan no puede ser exactamente 1)
    def no_adjacent(variables, values):
        r1, c1 = values[0]
        r2, c2 = values[1]
        return abs(r1 - r2) + abs(c1 - c2) != 1

    # Aplicamos "Sin superposición" a todos los pares
    for i in range(len(variables)):
        for j in range(i + 1, len(variables)):
            constraints.append(((variables[i], variables[j]), all_diff))

    gen_vars = [v for v in variables if v.startswith("gen")]
    hab_vars = [v for v in variables if v.startswith("hab")]

    # Generadores no adyacentes a Habitacionales
    for g in gen_vars:
        for h in hab_vars:
            constraints.append(((g, h), no_adjacent))

    # Generadores no adyacentes entre sí
    for i in range(len(gen_vars)):
        for j in range(i + 1, len(gen_vars)):
            constraints.append(((gen_vars[i], gen_vars[j]), no_adjacent))

    # Restricción: Laboratorio debe ser adyacente a al menos un depósito
    def lab_dep_adj(variables, values):
        lab_pos = values[0]
        dep_poses = values[1:]
        for r, c in dep_poses:
            if abs(lab_pos[0] - r) + abs(lab_pos[1] - c) == 1:
                return True
        return False

    lab_vars = [v for v in variables if v.startswith("lab")]
    dep_vars = [v for v in variables if v.startswith("dep")]

    for lab in lab_vars:
        constraints.append(([lab] + dep_vars, lab_dep_adj))

    # Restricción: Habitacional debe tener una ruta de evacuación libre
    def hab_free_adj(variables, values):
        hab_pos = values[0]
        other_poses = set(values[1:])
        r, c = hab_pos

        # Revisamos las 4 direcciones ortogonales
        neighbors = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]
        for nr, nc in neighbors:
            if 0 <= nr < R and 0 <= nc < C:
                if (nr, nc) not in craters_set and (nr, nc) not in other_poses:
                    return True
        return False

    for hab in hab_vars:
        other_vars = [v for v in variables if v != hab]
        # Esta restricción involucra al habitacional y a *todos* los demás módulos
        constraints.append(([hab] + other_vars, hab_free_adj))

    # 4. Resolución del CSP
    problem = CspProblem(variables, domains, constraints)

    # Se utiliza la heurística de Variable Más Restringida para acelerar el backtracking
    result = backtrack(problem, variable_heuristic=MOST_CONSTRAINED_VARIABLE)

    if result is None:
        return None

    # 5. Formatear la salida según las especificaciones
    final_distribution = []
    for var, pos in result.items():
        m_type = var.split('_')[0]
        final_distribution.append((m_type, pos[0], pos[1]))

    return final_distribution