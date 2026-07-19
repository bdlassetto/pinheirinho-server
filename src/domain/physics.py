import math

def dist_point_to_seg_2d(px, pz, ax, az, bx, bz):
    """
    Calcula a menor distancia em 2D (plano XZ) entre um ponto (pneu)
    e um segmento de reta (linha de stage diagonal).
    Compativel com Python 3.3.5. Sem type hints, sem f-strings.

    px, pz: Coordenadas X e Z do pneu
    ax, az: Coordenadas X e Z da fotocelula Esquerda (Start)
    bx, bz: Coordenadas X e Z da fotocelula Direita (End)
    """
    # Vetor do segmento AB (Linha da pista)
    ab_x = bx - ax
    ab_z = bz - az

    # Vetor do ponto A ao ponto P (Pneu)
    ap_x = px - ax
    ap_z = pz - az

    # Quadrado do comprimento do segmento AB
    ab_len_sq = ab_x * ab_x + ab_z * ab_z

    if ab_len_sq == 0:
        # Fallback caso os pontos A e B sejam identicos
        return math.sqrt(ap_x * ap_x + ap_z * ap_z)

    # Projecao escalar de AP sobre AB normalizada (encontra o escalar 't')
    t = (ap_x * ab_x + ap_z * ab_z) / ab_len_sq

    # Restringe 't' entre 0.0 e 1.0 para garantir que a projecao nao saia do segmento.
    # Isso resolve naturalmente o Lane Detect (ignora carros em outras faixas).
    t = max(0.0, min(1.0, t))

    # Encontra as coordenadas do ponto perfeitamente projetado no segmento
    proj_x = ax + t * ab_x
    proj_z = az + t * ab_z

    # Retorna a distancia do pneu ate esse ponto projetado na linha diagonal
    dx = px - proj_x
    dz = pz - proj_z

    return math.sqrt(dx * dx + dz * dz)

def dist_point_to_seg_2d_with_t(px, pz, ax, az, bx, bz):
    """
    Identico a dist_point_to_seg_2d, mas retorna (dist, t) onde:
      dist = distancia perpendicular ao segmento (metros)
      t    = escalar [0.0, 1.0] indicando a posicao lateral do ponto
             dentro da largura do segmento.
             t em [0,1] => o ponto esta dentro da largura da faixa.
             t < 0 ou t > 1 => ponto fora dos extremos laterais.

    Usado pela burnout box para validar se o carro esta na faixa certa.
    Compativel com Python 3.3.5.
    """
    ab_x = bx - ax
    ab_z = bz - az
    ap_x = px - ax
    ap_z = pz - az
    ab_len_sq = ab_x * ab_x + ab_z * ab_z

    if ab_len_sq == 0:
        return (math.sqrt(ap_x * ap_x + ap_z * ap_z), 0.0)

    t_raw = (ap_x * ab_x + ap_z * ab_z) / ab_len_sq
    t = max(0.0, min(1.0, t_raw))

    proj_x = ax + t * ab_x
    proj_z = az + t * ab_z
    dx = px - proj_x
    dz = pz - proj_z

    return (math.sqrt(dx * dx + dz * dz), t_raw)

def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def dist_sq(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return dx*dx + dy*dy + dz*dz

def scale_segment(start, end, factor):
    """
    Estica (ou encolhe) um segmento de reta 3D em torno do proprio centro.

    factor=1.10 alonga a linha em 10% (5% alem de cada extremidade),
    mantendo o ponto medio e a direcao. Usado para alargar a area util
    dos feixes de stage/pre-stage sem alterar sua posicao na pista.

    Retorna (new_start, new_end) como tuplas (x, y, z).
    Compativel com Python 3.3.5.
    """
    mx = (start[0] + end[0]) * 0.5
    my = (start[1] + end[1]) * 0.5
    mz = (start[2] + end[2]) * 0.5
    new_start = (mx + (start[0] - mx) * factor,
                 my + (start[1] - my) * factor,
                 mz + (start[2] - mz) * factor)
    new_end = (mx + (end[0] - mx) * factor,
               my + (end[1] - my) * factor,
               mz + (end[2] - mz) * factor)
    return (new_start, new_end)

def dist_point_to_seg(px, py, pz, a, b):
    x1, y1, z1 = a
    x2, y2, z2 = b
    vx, vy, vz = (x2-x1, y2-y1, z2-z1)
    wx, wy, wz = (px-x1, py-y1, pz-z1)
    ll = vx*vx + vy*vy + vz*vz
    if ll <= 1e-12:
        return 1e9
    t = (vx*wx + vy*wy + vz*wz) / ll
    t = max(0.0, min(1.0, t))
    cx, cy, cz = (x1 + t*vx, y1 + t*vy, z1 + t*vz)
    dx, dy, dz = (cx-px, cy-py, cz-pz)
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def lane_forward_unit_xz(stage_start, stage_end, pre_start, pre_end):
    # Calculates the forward vector of the lane based on stage and prestage lines
    # Using midpoints for stability
    s_mid = ((stage_start[0]+stage_end[0])*0.5, 0, (stage_start[2]+stage_end[2])*0.5)
    p_mid = ((pre_start[0]+pre_end[0])*0.5, 0, (pre_start[2]+pre_end[2])*0.5)
    
    vx = s_mid[0] - p_mid[0]
    vz = s_mid[2] - p_mid[2]
    ll = vx*vx + vz*vz
    if ll <= 1e-12:
        return (1.0, 0.0)
    inv = 1.0 / math.sqrt(ll)
    return (vx*inv, vz*inv)

def dist_along_forward_xz(pos, origin, forward_xz):
    fx, fz = forward_xz
    dx = pos[0] - origin[0]
    dz = pos[2] - origin[2]
    d = dx*fx + dz*fz
    return max(0.0, d)
