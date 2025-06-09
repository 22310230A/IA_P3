import networkx as nx
import matplotlib.pyplot as plt

# ===============================
# Grafo representando conexiones reales entre ciudades
# ===============================
# Cada clave representa una ciudad y contiene un diccionario de vecinos con pesos (minutos o costo estimado)
grafo = {
    'Guadalajara': {'Aguascalientes': 180, 'León': 210},
    'Aguascalientes': {'Guadalajara': 180, 'San Luis': 160},
    'León': {'Guadalajara': 210, 'Querétaro': 180},
    'San Luis': {'Aguascalientes': 160, 'Querétaro': 220},
    'Querétaro': {'San Luis': 220, 'León': 180, 'CDMX': 180},
    'CDMX': {'Querétaro': 180}
}

# ===============================
# Algoritmo de Prim paso a paso
# ===============================
def prim_visual(grafo, inicio):
    visitados = set()                      # Conjunto de ciudades visitadas
    aristas_mst = []                      # Lista final de aristas del árbol mínimo
    total = 0                              # Acumulador del costo total

    visitados.add(inicio)
    print(f"\n Iniciando en nodo: {inicio}")

    while len(visitados) < len(grafo):
        menor = None
        peso_menor = float('inf')

        # Buscar la arista más barata que conecte un nodo visitado con uno no visitado
        for nodo in visitados:
            for vecino, peso in grafo[nodo].items():
                if vecino not in visitados and peso < peso_menor:
                    menor = (nodo, vecino)
                    peso_menor = peso

        if menor is None:
            break  # Por si el grafo no está completamente conectado

        nodo1, nodo2 = menor
        visitados.add(nodo2)
        aristas_mst.append((nodo1, nodo2, peso_menor))
        total += peso_menor

        print(f"✔ Se agrega arista: {nodo1} — {nodo2} (peso: {peso_menor})")
        print(f"  Nodos visitados: {sorted(visitados)}")

    print(f"\nÁrbol de expansión mínima completado. Costo total: {total}")
    return aristas_mst

# ===============================
# Gráfico del Árbol Parcial Mínimo
# ===============================
def graficar_prim(grafo, aristas_mst):
    G = nx.Graph()

    # Construir el grafo completo con pesos
    for nodo in grafo:
        for vecino, peso in grafo[nodo].items():
            G.add_edge(nodo, vecino, weight=peso)

    pos = nx.spring_layout(G, seed=42)
    edge_labels = nx.get_edge_attributes(G, 'weight')

    plt.figure(figsize=(10, 6))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=1000, font_size=11)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    # Extraer solo las aristas del MST
    aristas_verdes = [(a, b) for a, b, _ in aristas_mst]
    nx.draw_networkx_edges(G, pos, edgelist=aristas_verdes, width=3, edge_color='green')

    plt.text(0, -1.1,
             'Aristas verdes: Árbol Parcial Mínimo generado con Prim\nCostos estimados en minutos entre ciudades',
             fontsize=10, ha='center', color='gray')

    plt.title("Árbol de Expansión Mínima con Prim - Ciudades Reales")
    plt.axis('off')
    plt.tight_layout()
    plt.show()

# ===============================
# Ejecutar todo
# ===============================
inicio = 'Guadalajara'
aristas_prim = prim_visual(grafo, inicio)
graficar_prim(grafo, aristas_prim)
