#!/usr/bin/env python3
"""Convert ros2_node_catalog.md into LaTeX chapter body."""
import re
from pathlib import Path

src = Path('/home/robot/simulation_ws/version_3/docs/ros2_node_catalog.md')
out_dir = Path('/home/robot/simulation_ws/docs/overleaf_magnetic_asv')
chapters = out_dir / 'chapters'
chapters.mkdir(parents=True, exist_ok=True)

text = src.read_text(encoding='utf-8')
text = text.replace('\u2014', '---').replace('\u2013', '--').replace('\u2019', "'")
text = text.replace('\u201c', '"').replace('\u201d', '"')


def inline(s):
    def code_sub(m):
        inner = m.group(1)
        for a, b in [
            ('\\', r'\textbackslash{}'),
            ('&', r'\&'),
            ('%', r'\%'),
            ('#', r'\#'),
            ('_', r'\_'),
            ('{', r'\{'),
            ('}', r'\}'),
            ('~', r'\textasciitilde{}'),
            ('^', r'\textasciicircum{}'),
        ]:
            inner = inner.replace(a, b)
        return r'\texttt{' + inner + '}'

    s = re.sub(r'`([^`]+)`', code_sub, s)
    s = re.sub(r'\*\*([^*]+)\*\*', lambda m: r'\textbf{' + m.group(1) + '}', s)
    s = re.sub(r'(?<!\*)\*([^*]+)\*(?!\*)', lambda m: r'\emph{' + m.group(1) + '}', s)
    for ch, rep in [('&', r'\&'), ('%', r'\%'), ('#', r'\#'), ('_', r'\_')]:
        s = re.sub(r'(?<!\\)' + re.escape(ch), rep, s)
    return s


lines = text.splitlines()
out = []
in_code = False
table_rows = []


def flush_table():
    global table_rows
    if not table_rows:
        return
    rows = []
    for row in table_rows:
        if re.match(r'^\s*\|?\s*:?-+:?\s*(\|\s*:?-+:?\s*)+\|?\s*$', row):
            continue
        cells = [c.strip() for c in row.strip().strip('|').split('|')]
        rows.append(cells)
    table_rows = []
    if not rows:
        return
    ncols = max(len(r) for r in rows)
    for r in rows:
        while len(r) < ncols:
            r.append('')
    if ncols == 2:
        colspec = 'lp{10cm}'
    elif ncols == 3:
        colspec = 'llp{7.2cm}'
    elif ncols == 4:
        colspec = 'lllp{5.2cm}'
    else:
        colspec = 'p{2.1cm}' * (ncols - 1) + 'p{3.2cm}'
    out.append(r'\begin{center}')
    out.append(r'\small')
    out.append(r'\begin{longtable}{@{}' + colspec + r'@{}}')
    out.append(r'\toprule')
    out.append(' & '.join(inline(c) for c in rows[0]) + r' \\')
    out.append(r'\midrule')
    out.append(r'\endfirsthead')
    out.append(r'\toprule')
    out.append(' & '.join(inline(c) for c in rows[0]) + r' \\')
    out.append(r'\midrule')
    out.append(r'\endhead')
    for r in rows[1:]:
        out.append(' & '.join(inline(c) for c in r) + r' \\')
    out.append(r'\bottomrule')
    out.append(r'\end{longtable}')
    out.append(r'\end{center}')
    out.append('')


i = 0
while i < len(lines):
    line = lines[i]
    if line.strip().startswith('```'):
        if not in_code:
            in_code = True
            out.append(r'\begin{verbatim}')
        else:
            in_code = False
            out.append(r'\end{verbatim}')
        i += 1
        continue
    if in_code:
        out.append(line)
        i += 1
        continue
    if '|' in line and line.strip().startswith('|'):
        table_rows.append(line)
        i += 1
        if i >= len(lines) or '|' not in lines[i] or not lines[i].strip().startswith('|'):
            flush_table()
        continue
    if table_rows:
        flush_table()

    if line.startswith('# '):
        title = inline(line[2:].strip())
        out.append(r'\chapter{' + title + '}')
        out.append(r'\label{ch:ros-catalog}')
        out.append('')
        out.append(
            'This chapter is the authoritative software reference for every '
            'message, bridge, and node in the dual-ASV stack. Each node section '
            'lists identity, parameters, subscriptions, publications, timers, '
            'QoS, internal algorithm steps, and runtime dependencies.'
        )
        out.append('')
    elif line.startswith('## '):
        out.append(r'\section{' + inline(line[3:].strip()) + '}')
        out.append('')
    elif line.startswith('### '):
        out.append(r'\subsection{' + inline(line[4:].strip()) + '}')
        out.append('')
    elif line.startswith('#### '):
        out.append(r'\subsubsection{' + inline(line[5:].strip()) + '}')
        out.append('')
    elif line.strip() == '---':
        out.append('')
    elif line.startswith('- '):
        items = []
        while i < len(lines) and lines[i].startswith('- '):
            items.append(inline(lines[i][2:].strip()))
            i += 1
        out.append(r'\begin{itemize}[leftmargin=*]')
        for it in items:
            out.append(r'\item ' + it)
        out.append(r'\end{itemize}')
        out.append('')
        continue
    elif re.match(r'^\d+\.\s', line.strip()):
        items = []
        while i < len(lines) and re.match(r'^\d+\.\s', lines[i].strip()):
            items.append(inline(re.sub(r'^\d+\.\s', '', lines[i].strip())))
            i += 1
        out.append(r'\begin{enumerate}[leftmargin=*]')
        for it in items:
            out.append(r'\item ' + it)
        out.append(r'\end{enumerate}')
        out.append('')
        continue
    elif line.strip() == '':
        out.append('')
    else:
        out.append(inline(line.strip()))
        out.append('')
    i += 1

if table_rows:
    flush_table()

path = chapters / 'ros_node_catalog.tex'
path.write_text('\n'.join(out), encoding='utf-8')
print('OK', path, 'lines', len(out))
