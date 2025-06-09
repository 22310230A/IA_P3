import networkx as nx
import matplotlib.pyplot as plt
import heapq

# ===============================
# Grafo representando rutas reales
# ===============================
# Cada clave es una ciudad, y cada valor es una lista de tuplas con ciudades vecinas y el tiempo estimado para llegar
grafo = {
    'Guadalajara': [('Aguascalientes', 180), ('León', 210)],
    'Aguascalientes': [('Guadalajara', 180), ('San Luis', 160)],
    'León': [('Guadalajara', 210), ('Querétaro', 180)],
    'San Luis': [('Aguascalientes', 160), ('Querétaro', 220)],
    'Querétaro': [('San Luis', 220), ('León', 180), ('CDMX', 180)],
    'CDMX': [('Querétaro', 180)]
}

# ===============================
# Algoritmo de Dijkstra
# ===============================
def dijkstra_visual(grafo, inicio):
    # Inicializar distancias a infinito y la del nodo inicial a 0
    distancias = {nodo: float('inf') for nodo in grafo}
    distancias[inicio] = 0

    # Conjunto de nodos ya visitados
    visitados = set()

    # Cola de prioridad con (distancia, nodo)
    cola = [(0, inicio)]

    # Diccionario para reconstruir el camino
    caminos = {}

    # Bucle principal
    while cola:
        distancia_actual, nodo_actual = heapq.heappop(cola)

        # Saltar si ya se visitó
        if nodo_actual in visitados:
            continue
        visitados.add(nodo_actual)

        print(f"\n Visitando: {nodo_actual} (Distancia acumulada: {distancia_actual})")

        # Revisar todos los vecinos del nodo actual
        for vecino, peso in grafo[nodo_actual]:
            nueva_distancia = distancia_actual + peso

            # Si encontramos una mejor ruta al vecino, la actualizamos
            if nueva_distancia < distancias[vecino]:
                print(f"  ↳ Mejor camino a {vecino} encontrado: {distancias[vecino]} → {nueva_distancia}")
                distancias[vecino] = nueva_distancia
                caminos[vecino] = nodo_actual
                heapq.heappush(cola, (nueva_distancia, vecino))

    return distancias, caminos

# ===============================
# Graficar el grafo con ruta más corta
# ===============================
def graficar_dijkstra(grafo, distancias, caminos, inicio, destino_final):
    G = nx.DiGraph()  # Grafo dirigido

    # Agregar nodos y aristas al grafo
    for nodo in grafo:
        for vecino, peso in grafo[nodo]:
            G.add_edge(nodo, vecino, weight=peso)

    # Posiciones visuales de los nodos
    pos = nx.spring_layout(G, seed=42)

    # Etiquetas de los pesos de las aristas
    edge_labels = nx.get_edge_attributes(G, 'weight')

    # Dibujar el grafo completo
    plt.figure(figsize=(10, 6))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=1000, font_size=11, arrows=True)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    # Reconstruir la ruta más corta desde el destino hasta el inicio
    ruta_corta = []
    actual = destino_final
    while actual in caminos:
        anterior = caminos[actual]
        ruta_corta.insert(0, (anterior, actual))  # Insertar al principio para mantener orden correcto
        actual = anterior

    # Dibujar la ruta más corta en color verde
    nx.draw_networkx_edges(G, pos, edgelist=ruta_corta, width=3, edge_color='green')

    # Agregar texto explicativo
    plt.text(0, -1.1,
             f'Ruta más corta desde {inicio} hasta {destino_final} mostrada en verde\nCostos: tiempo estimado entre ciudades',
             fontsize=10, ha='center', color='gray')

    plt.title(f"Ruta más rápida de {inicio} a {destino_final} con Dijkstra")
    plt.axis('off')
    plt.tight_layout()
    plt.show()

# ===============================
# Ejecutar todo
# ===============================
inicio = 'Guadalajara'
destino_final = 'CDMX'
distancias, caminos = dijkstra_visual(grafo, inicio)
graficar_dijkstra(grafo, distancias, caminos, inicio, destino_final)
