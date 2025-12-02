import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ==============================================================================
# ETAPA 1: COLETA DE DADOS DO USUÁRIO
# ==============================================================================
def coletar_dados():
    """Coleta os dados necessários do usuário"""
    print("=" * 60)
    print("SISTEMA DE PROJEÇÃO PERSPECTIVA - PIRÂMIDE 3D")
    print("=" * 60)
    
    # Ponto de vista (olho/câmera)
    print("\n📷 PONTO DE VISTA (Câmera/Olho):")
    print("Exemplo: 0 0 20")
    entrada = input("Digite as coordenadas C (a b c): ").split()
    C = np.array([float(entrada[0]), float(entrada[1]), float(entrada[2])])
    
    # Pontos do plano de projeção
    print("\n🖼️  PLANO DE PROJEÇÃO (3 pontos não colineares):")
    print("Exemplo de plano XY em z=0:")
    print("P1: 0 0 0")
    print("P2: 10 0 0")
    print("P3: 0 10 0")
    
    entrada = input("\nDigite P1 (x y z): ").split()
    P1 = np.array([float(entrada[0]), float(entrada[1]), float(entrada[2])])
    
    entrada = input("Digite P2 (x y z): ").split()
    P2 = np.array([float(entrada[0]), float(entrada[1]), float(entrada[2])])
    
    entrada = input("Digite P3 (x y z): ").split()
    P3 = np.array([float(entrada[0]), float(entrada[1]), float(entrada[2])])
    
    # Vértices da pirâmide 3D
    print("\n🔺 PIRÂMIDE 3D (Base quadrada + Ápice = 5 vértices):")
    print("Exemplo (pirâmide com base 6x6 no plano z=5):")
    print("Base:")
    print("  V1: -3 -3 5  (canto inferior esquerdo)")
    print("  V2:  3 -3 5  (canto inferior direito)")
    print("  V3:  3  3 5  (canto superior direito)")
    print("  V4: -3  3 5  (canto superior esquerdo)")
    print("Ápice:")
    print("  V5:  0  0 12 (topo da pirâmide)")
    
    piramide = []
    for i in range(5):
        if i < 4:
            entrada = input(f"\nDigite V{i+1} - Base (x y z): ").split()
        else:
            entrada = input(f"\nDigite V{i+1} - Ápice (x y z): ").split()
        vertice = np.array([float(entrada[0]), float(entrada[1]), float(entrada[2])])
        piramide.append(vertice)
    
    return C, P1, P2, P3, piramide


# ==============================================================================
# ETAPA 2: CALCULAR VETOR NORMAL DO PLANO
# ==============================================================================
def calcular_vetor_normal(P1, P2, P3):
    """Calcula o vetor normal ao plano usando produto vetorial."""
    print("\n" + "=" * 60)
    print("ETAPA 2: CÁLCULO DO VETOR NORMAL")
    print("=" * 60)
    
    V1 = P1 - P2
    V2 = P3 - P2
    
    print(f"\nVetor V1 (P1-P2) = {V1}")
    print(f"Vetor V2 (P3-P2) = {V2}")
    
    Nx = V1[1] * V2[2] - V2[1] * V1[2]
    Ny = -(V1[0] * V2[2] - V2[0] * V1[2])
    Nz = V1[0] * V2[1] - V2[0] * V1[1]
    
    N = np.array([Nx, Ny, Nz])
    
    print(f"\n✓ Vetor Normal N = {N}")
    print(f"  (Perpendicular ao plano de projeção)")
    
    return N


# ==============================================================================
# ETAPA 3: CALCULAR d0, d1 e d
# ==============================================================================
def calcular_constantes(P1, C, N):
    """Calcula as constantes d0, d1 e d."""
    print("\n" + "=" * 60)
    print("ETAPA 3: CÁLCULO DAS CONSTANTES d0, d1 e d")
    print("=" * 60)
    
    d0 = P1[0] * N[0] + P1[1] * N[1] + P1[2] * N[2]
    print(f"\nd0 = {d0:.3f}")
    print(f"  → 'Distância' do plano à origem")
    
    d1 = C[0] * N[0] + C[1] * N[1] + C[2] * N[2]
    print(f"\nd1 = {d1:.3f}")
    print(f"  → 'Distância' do olho à origem")
    
    d = d0 - d1
    print(f"\nd = {d:.3f}")
    print(f"  → Distância relativa olho-plano")
    
    if d > 0:
        print(f"  ✓ d > 0: Configuração normal de visualização")
    
    return d0, d1, d


# ==============================================================================
# ETAPA 4: MONTAR MATRIZ DE PERSPECTIVA
# ==============================================================================
def criar_matriz_perspectiva(C, N, d, d0):
    """Cria a matriz de transformação perspectiva 4x4."""
    print("\n" + "=" * 60)
    print("ETAPA 4: MONTAGEM DA MATRIZ DE PERSPECTIVA")
    print("=" * 60)
    
    a, b, c = C[0], C[1], C[2]
    Nx, Ny, Nz = N[0], N[1], N[2]
    
    M = np.array([
        [d + a*Nx,    a*Ny,        a*Nz,        -a*d0],
        [b*Nx,        d + b*Ny,    b*Nz,        -b*d0],
        [c*Nx,        c*Ny,        d + c*Nz,    -c*d0],
        [Nx,          Ny,          Nz,          1    ]
    ])
    
    print("\n✓ Matriz de Perspectiva 4x4 criada")
    
    return M


# ==============================================================================
# ETAPA 5: PROJETAR PONTOS DA PIRÂMIDE
# ==============================================================================
def projetar_pontos(piramide, M):
    """Projeta cada vértice da pirâmide no plano."""
    print("\n" + "=" * 60)
    print("ETAPA 5: PROJEÇÃO DOS VÉRTICES DA PIRÂMIDE")
    print("=" * 60)
    
    pontos_projetados = []
    
    for i, ponto in enumerate(piramide):
        nome = f"Base V{i+1}" if i < 4 else "Ápice V5"
        print(f"\n--- {nome} = {ponto} ---")
        
        P_homogeneo = np.array([ponto[0], ponto[1], ponto[2], 1])
        P_projetado = M @ P_homogeneo
        
        W_prime = P_projetado[3]
        XC = P_projetado[0] / W_prime
        YC = P_projetado[1] / W_prime
        ZC = P_projetado[2] / W_prime
        
        XP = XC
        YP = YC
        print(f"✓ Projetado: ({XP:.3f}, {YP:.3f})")
        
        pontos_projetados.append([XP, YP])
    
    return np.array(pontos_projetados)


# ==============================================================================
# ETAPA 6: TRANSFORMAÇÃO JANELA-VIEWPORT
# ==============================================================================
def transformar_viewport(pontos_projetados, largura_tela=800, altura_tela=600):
    """Transforma coordenadas do plano para coordenadas da tela."""
    print("\n" + "=" * 60)
    print("ETAPA 6: TRANSFORMAÇÃO JANELA → VIEWPORT")
    print("=" * 60)
    
    XP_coords = pontos_projetados[:, 0]
    YP_coords = pontos_projetados[:, 1]
    
    xmin_obj = np.min(XP_coords)
    xmax_obj = np.max(XP_coords)
    ymin_obj = np.min(YP_coords)
    ymax_obj = np.max(YP_coords)
    
    largura = xmax_obj - xmin_obj
    altura = ymax_obj - ymin_obj
    margem = 0.15
    
    xmin = xmin_obj - largura * margem
    xmax = xmax_obj + largura * margem
    ymin = ymin_obj - altura * margem
    ymax = ymax_obj + altura * margem
    
    print(f"\n📐 JANELA: x ∈ [{xmin:.2f}, {xmax:.2f}], y ∈ [{ymin:.2f}, {ymax:.2f}]")
    
    umin = 0
    umax = largura_tela
    vmin = 0
    vmax = altura_tela
    
    print(f"🖥️  VIEWPORT: {largura_tela} x {altura_tela} pixels")
    
    Rw = (xmax - xmin) / (ymax - ymin)
    Rv = (umax - umin) / (vmax - vmin)
    
    sx = (umax - umin) / (xmax - xmin)
    sy = (vmax - vmin) / (ymax - ymin)
    
    pontos_tela = []
    
    if Rw > Rv:
        sy_novo = sx / Rw
        vmax_novo = vmin + (ymax - ymin) * sy_novo
        v_offset = (vmax - vmax_novo) / 2 + vmin
        
        for XP, YP in pontos_projetados:
            u = sx * (XP - xmin) + umin
            v = -sy_novo * (YP - ymax) + v_offset
            pontos_tela.append([u, v])
    else:
        sx_novo = sy * Rw
        umax_novo = umin + (xmax - xmin) * sx_novo
        u_offset = (umax - umax_novo) / 2 + umin
        
        for XP, YP in pontos_projetados:
            u = sx_novo * (XP - xmin) + u_offset
            v = -sy * (YP - ymax) + vmin
            pontos_tela.append([u, v])
    
    print(f"\n✓ Transformação concluída - {len(pontos_tela)} vértices")
    
    return np.array(pontos_tela), (xmin, xmax, ymin, ymax)


# ==============================================================================
# ETAPA 7: VISUALIZAÇÃO 3D + PROJEÇÃO 2D
# ==============================================================================
def desenhar_resultado(piramide, pontos_projetados, pontos_tela, C, P1, janela, largura=800, altura=600):
    """Desenha a pirâmide 3D e sua projeção 2D."""
    print("\n" + "=" * 60)
    print("ETAPA 7: VISUALIZAÇÃO DO RESULTADO")
    print("=" * 60)
    
    fig = plt.figure(figsize=(18, 6))
    
    # ========== GRÁFICO 1: CENA 3D ==========
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.set_title('Cena 3D Original', fontsize=12, fontweight='bold')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    
    # Desenhar pirâmide 3D
    vertices = np.array(piramide)
    
    # Definir as faces da pirâmide (base + 4 faces laterais)
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # Base
        [vertices[0], vertices[1], vertices[4]],  # Face frontal
        [vertices[1], vertices[2], vertices[4]],  # Face direita
        [vertices[2], vertices[3], vertices[4]],  # Face traseira
        [vertices[3], vertices[0], vertices[4]],  # Face esquerda
    ]
    
    # Criar coleção de polígonos 3D
    poly = Poly3DCollection(faces, alpha=0.6, facecolor='cyan', edgecolor='darkblue', linewidth=2)
    ax1.add_collection3d(poly)
    
    # Plotar vértices
    ax1.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], 
                c='red', s=100, marker='o', edgecolors='black', linewidths=2)
    
    # Labels dos vértices
    for i, v in enumerate(vertices):
        label = f'V{i+1}'
        ax1.text(v[0], v[1], v[2], f'  {label}', fontsize=9)
    
    # Ponto de vista
    ax1.scatter([C[0]], [C[1]], [C[2]], c='yellow', s=200, marker='*', 
                edgecolors='black', linewidths=2, label='Câmera')
    ax1.text(C[0], C[1], C[2], '  👁️ C', fontsize=11, fontweight='bold')
    
    # Plano de projeção (indicativo)
    ax1.scatter([P1[0]], [P1[1]], [P1[2]], c='green', s=100, marker='s', 
                edgecolors='black', linewidths=2, label='Plano')
    
    ax1.legend()
    
    # ========== GRÁFICO 2: PROJEÇÃO NO PLANO (MUNDO) ==========
    ax2 = fig.add_subplot(132)
    xmin, xmax, ymin, ymax = janela
    ax2.set_xlim(xmin, xmax)
    ax2.set_ylim(ymin, ymax)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Projeção no Plano (Mundo)', fontsize=12, fontweight='bold')
    ax2.set_xlabel('XP')
    ax2.set_ylabel('YP')
    
    # Desenhar a projeção da pirâmide
    # Base projetada
    base_proj = plt.Polygon(pontos_projetados[:4], fill=True, facecolor='lightgreen', 
                            edgecolor='darkgreen', linewidth=2, alpha=0.5)
    ax2.add_patch(base_proj)
    
    # Linhas do ápice para a base
    for i in range(4):
        ax2.plot([pontos_projetados[i][0], pontos_projetados[4][0]], 
                [pontos_projetados[i][1], pontos_projetados[4][1]], 
                'b-', linewidth=1.5)
    
    # Vértices
    for i, (x, y) in enumerate(pontos_projetados):
        cor = 'red' if i < 4 else 'darkred'
        tamanho = 8 if i < 4 else 12
        ax2.plot(x, y, 'o', color=cor, markersize=tamanho)
        ax2.text(x, y, f'  V{i+1}', fontsize=9, verticalalignment='bottom')
    
    # ========== GRÁFICO 3: VIEWPORT (TELA) ==========
    ax3 = fig.add_subplot(133)
    ax3.set_xlim(0, largura)
    ax3.set_ylim(altura, 0)
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Viewport (Tela em Pixels)', fontsize=12, fontweight='bold')
    ax3.set_xlabel('u (pixels)')
    ax3.set_ylabel('v (pixels)')
    
    # Base projetada na tela
    base_tela = plt.Polygon(pontos_tela[:4], fill=True, facecolor='lightblue', 
                            edgecolor='darkblue', linewidth=2, alpha=0.6)
    ax3.add_patch(base_tela)
    
    # Linhas do ápice para a base
    for i in range(4):
        ax3.plot([pontos_tela[i][0], pontos_tela[4][0]], 
                [pontos_tela[i][1], pontos_tela[4][1]], 
                'darkblue', linewidth=2)
    
    # Vértices
    for i, (u, v) in enumerate(pontos_tela):
        cor = 'blue' if i < 4 else 'darkred'
        tamanho = 8 if i < 4 else 12
        ax3.plot(u, v, 'o', color=cor, markersize=tamanho)
        ax3.text(u, v, f'  V{i+1}', fontsize=9, verticalalignment='top')
    
    plt.tight_layout()
    plt.savefig('projecao_piramide.png', dpi=150, bbox_inches='tight')
    print("\n✓ Imagem salva como 'projecao_piramide.png'")
    plt.show()


# ==============================================================================
# FUNÇÃO PRINCIPAL
# ==============================================================================
def main():
    """Executa o pipeline completo de projeção perspectiva."""
    
    # ETAPA 1: Coletar dados
    C, P1, P2, P3, piramide = coletar_dados()
    
    # ETAPA 2: Calcular vetor normal
    N = calcular_vetor_normal(P1, P2, P3)
    
    # ETAPA 3: Calcular constantes
    d0, d1, d = calcular_constantes(P1, C, N)
    
    # ETAPA 4: Criar matriz de perspectiva
    M = criar_matriz_perspectiva(C, N, d, d0)
    
    # ETAPA 5: Projetar pontos
    pontos_projetados = projetar_pontos(piramide, M)
    
    # ETAPA 6: Transformar para viewport
    pontos_tela, janela = transformar_viewport(pontos_projetados)
    
    # ETAPA 7: Desenhar resultado
    desenhar_resultado(piramide, pontos_projetados, pontos_tela, C, P1, janela)
    
    print("\n" + "=" * 60)
    print("✅ PROCESSAMENTO CONCLUÍDO!")
    print("=" * 60)


# ==============================================================================
# EXECUÇÃO
# ==============================================================================
if __name__ == "__main__":
    print("\n🔺 PROJEÇÃO PERSPECTIVA DE PIRÂMIDE 3D")
    print("\n💡 DICA: Use estes valores de exemplo para testar:")
    print("\nPonto de Vista: 0 0 20")
    print("\nPlano (XY em z=0):")
    print("P1: 0 0 0")
    print("P2: 10 0 0")
    print("P3: 0 10 0")
    print("\nPirâmide (Base 6x6 no plano z=5):")
    print("V1: -3 -3 5")
    print("V2:  3 -3 5")
    print("V3:  3  3 5")
    print("V4: -3  3 5")
    print("V5:  0  0 12  (ápice)")
    print("\n" + "-" * 60 + "\n")
    
    main()