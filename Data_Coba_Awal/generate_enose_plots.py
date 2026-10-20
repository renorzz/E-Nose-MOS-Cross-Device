"""
================================================================================
E-Nose Multi-Sensor Time Series Plotting Script (IEEE Standard Style)
Generates publication-quality response plots for 8 sensors (C1-C8)
across 5 devices from time series 0-360 seconds.
================================================================================
"""

import os
import shutil
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from scipy.signal import savgol_filter

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, "Hasil_Plot")
ARTIFACT_DIR = "/Users/moreno/.gemini/antigravity-ide/brain/a8ea2350-14a8-4269-b3e0-1506c5e78201"
os.makedirs(OUTPUT_DIR, exist_ok=True)

CHANNELS = [
    {'id': 'C1', 'col': 'MQ6',     'name': 'MQ-6',     'target': 'CH₄ / LPG',         'color': '#DC2626'},
    {'id': 'C2', 'col': 'MQ135',   'name': 'MQ-135',   'target': 'Air Quality / NH₃', 'color': '#EA580C'},
    {'id': 'C3', 'col': 'MQ3',     'name': 'MQ-3',     'target': 'Alcohol / Ethanol',  'color': '#D97706'},
    {'id': 'C4', 'col': 'MQ7',     'name': 'MQ-7',     'target': 'Carbon Monoxide',   'color': '#16A34A'},
    {'id': 'C5', 'col': 'TGS2611', 'name': 'TGS-2611', 'target': 'Methane / Nat. Gas','color': '#0284C7'},
    {'id': 'C6', 'col': 'TGS2602', 'name': 'TGS-2602', 'target': 'VOC / Odor / H₂S',  'color': '#4F46E5'},
    {'id': 'C7', 'col': 'TGS2600', 'name': 'TGS-2600', 'target': 'Air Contaminants',  'color': '#7C3AED'},
    {'id': 'C8', 'col': 'TGS2620', 'name': 'TGS-2620', 'target': 'Solvents / Vapors', 'color': '#DB2777'},
]

DEVICES = [
    {'id': 'D1', 'name': 'Device 1', 'file': 'device1.csv', 'color': '#DC2626'},
    {'id': 'D2', 'name': 'Device 2', 'file': 'device2.csv', 'color': '#EA580C'},
    {'id': 'D3', 'name': 'Device 3', 'file': 'device3.csv', 'color': '#2563EB'},
    {'id': 'D4', 'name': 'Device 4', 'file': 'device4.csv', 'color': '#059669'},
    {'id': 'D5', 'name': 'Device 5', 'file': 'device5.csv', 'color': '#7C3AED'},
]

TIME_GRID = np.arange(0, 361, 1.0) # 0 to 360 seconds (361 points)

def load_and_preprocess(filepath):
    df = pd.read_csv(filepath, sep=';', decimal=',')
    resets = df[df['Elapsed Time (s)'].diff() < 0].index.tolist()
    first_reset = resets[0] if len(resets) > 0 else len(df)
    c = df.iloc[:first_reset].copy()
    c = c[(c['Elapsed Time (s)'] >= 0) & (c['Elapsed Time (s)'] <= 360)]
    c_clean = c.groupby('Elapsed Time (s)').mean().reset_index()
    t_raw = c_clean['Elapsed Time (s)'].values
    
    processed = {'time': TIME_GRID}
    for ch in CHANNELS:
        col = ch['col']
        v_raw = np.interp(TIME_GRID, t_raw, c_clean[f'{col} (mV)'].values)
        v_filt = savgol_filter(v_raw, window_length=15, polyorder=2)
        v0 = np.mean(v_filt[:10])
        if v0 == 0: v0 = 1e-6
        dv_v0 = (v_filt - v0) / v0
        
        rsr0_raw = np.interp(TIME_GRID, t_raw, c_clean[f'{col}_RsR0'].values)
        rsr0_filt = savgol_filter(rsr0_raw, window_length=15, polyorder=2)
        
        processed[f'{col}_mV_raw'] = v_raw
        processed[f'{col}_mV_filt'] = v_filt
        processed[f'{col}_norm'] = dv_v0
        processed[f'{col}_RsR0'] = rsr0_filt
        processed[f'{col}_V0'] = v0
        
    return processed

# Load data
print("Loading and preprocessing all 5 device datasets...")
device_data = {}
for dev in DEVICES:
    fp = os.path.join(BASE_DIR, dev['file'])
    device_data[dev['id']] = load_and_preprocess(fp)
    print(f"  Processed {dev['name']}")

def apply_plot_style():
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.sans-serif': ['Helvetica', 'Arial', 'DejaVu Sans'],
        'axes.edgecolor': '#0F172A',
        'axes.linewidth': 1.1,
        'xtick.direction': 'in',
        'ytick.direction': 'in',
        'xtick.major.size': 4.0,
        'ytick.major.size': 4.0,
        'xtick.color': '#0F172A',
        'ytick.color': '#0F172A',
        'figure.dpi': 300,
    })

letters = 'abcdefgh'

# ─────────────────────────────────────────────────────────────────────────────
# 1. FIVE PLOTS: 4x2 Grid Plot for each Device (Device 1 to Device 5)
# ─────────────────────────────────────────────────────────────────────────────
def generate_device_grid_plots():
    apply_plot_style()
    output_files = []
    
    for dev_idx, dev in enumerate(DEVICES):
        dev_id = dev['id']
        data = device_data[dev_id]
        
        fig = plt.figure(figsize=(22, 12), facecolor='white')
        gs = gridspec.GridSpec(2, 4, figure=fig, hspace=0.36, wspace=0.25,
                               top=0.81, bottom=0.08, left=0.06, right=0.97)
        
        for i, ch in enumerate(CHANNELS):
            row, col = divmod(i, 4)
            ax = fig.add_subplot(gs[row, col])
            ax.set_facecolor('white')
            
            c_col = ch['col']
            y_norm = data[f'{c_col}_norm']
            y_raw = (data[f'{c_col}_mV_raw'] - data[f'{c_col}_V0']) / data[f'{c_col}_V0']
            
            # Baseline Shading (0 to 70s)
            ax.axvspan(0, 70, color='#F1F5F9', alpha=0.9, zorder=0)
            ax.axhline(0, color='#94A3B8', linewidth=0.8, linestyle='-', alpha=0.7, zorder=1)
            
            # Raw signal (thin subtle) + Filtered signal (bold primary)
            ax.plot(TIME_GRID, y_raw, color=ch['color'], linewidth=0.8, alpha=0.35, zorder=2, label='Raw ADC')
            ax.plot(TIME_GRID, y_norm, color=ch['color'], linewidth=2.2, alpha=0.95, zorder=3, label='SG Filtered')
            
            # Inlet transition line at 70s
            ax.axvline(70, color='#64748B', linestyle=':', linewidth=1.2, alpha=0.85, zorder=4)
            
            # Subplot title
            ax.set_title(f'({letters[i]}) {ch["id"]} Normalized Response\n{ch["name"]} ({ch["target"]})',
                         fontsize=10.5, fontweight='bold', color='#0F172A', pad=6)
            
            ax.set_xlabel('Time (seconds)', fontsize=9.5, fontweight='bold', color='#1E293B')
            ax.set_ylabel(r'$\Delta V / V_0\ (-)$', fontsize=10, fontweight='bold', color='#1E293B')
            
            ax.set_xlim(0, 360)
            ax.set_xticks([0, 60, 120, 180, 240, 300, 360])
            
            y_min = min(np.nanmin(y_norm), -0.05)
            y_max = max(np.nanmax(y_norm), 0.1)
            margin = max((y_max - y_min) * 0.15, 0.04)
            ax.set_ylim(y_min - margin, y_max + margin)
            
            ax.tick_params(axis='both', labelsize=8.5, colors='#1E293B')
            ax.grid(True, linestyle='--', alpha=0.3, color='#CBD5E1', zorder=0)
            
            for sp in ax.spines.values():
                sp.set_edgecolor('#0F172A')
                sp.set_linewidth(1.0)
                
            ax.text(73, ax.get_ylim()[1] * 0.88, 'Inlet\n70s', fontsize=7.5,
                    color='#64748B', ha='left', va='top', style='italic')
        
        # Super title with ample room
        fig.suptitle(
            f'Comprehensive 8-Channel Multi-Sensor Responses (Channels C1–C8) — {dev["name"]}\n'
            f'Gas Response Profiles (MQ-6, MQ-135, MQ-3, MQ-7, TGS-2611, TGS-2602, TGS-2600, TGS-2620) | 0–360s, IEEE Standard Style',
            fontsize=13.5, fontweight='bold', color='#0F172A', y=0.965, va='top'
        )
        
        # Legend positioned cleanly in gap between suptitle and subplots
        legend_handles = [
            Line2D([0], [0], color='#475569', linewidth=1.0, alpha=0.5, label='Raw ADC Data'),
            Line2D([0], [0], color=dev['color'], linewidth=2.4, label='Savitzky-Golay Filtered ($\Delta V/V_0$)'),
            Line2D([0], [0], color='#64748B', linestyle=':', linewidth=1.5, label='Inlet Onset (t = 70s)'),
        ]
        fig.legend(handles=legend_handles, loc='upper center', ncol=3,
                   fontsize=10, frameon=True, framealpha=0.95, edgecolor='#CBD5E1',
                   bbox_to_anchor=(0.5, 0.885))
        
        out_name = f'plot_device{dev_idx+1}_grid4x2.png'
        out_path = os.path.join(OUTPUT_DIR, out_name)
        fig.savefig(out_path, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close(fig)
        shutil.copy(out_path, os.path.join(ARTIFACT_DIR, out_name))
        output_files.append(out_path)
        print(f"[+] Saved Grid 4x2 Plot: {out_name}")
        
    return output_files

# ─────────────────────────────────────────────────────────────────────────────
# 2. FIVE PLOTS: Combined Single-Axis 8-Sensor Plot for each Device
# ─────────────────────────────────────────────────────────────────────────────
def generate_device_combined_plots():
    apply_plot_style()
    output_files = []
    
    for dev_idx, dev in enumerate(DEVICES):
        dev_id = dev['id']
        data = device_data[dev_id]
        
        fig, ax = plt.subplots(figsize=(15, 8.5), facecolor='white')
        plt.subplots_adjust(top=0.79, bottom=0.10, left=0.08, right=0.96)
        
        # Operational phases
        ax.axvspan(0, 70, color='#F8FAFC', alpha=0.9, zorder=0)
        ax.axvspan(70, 170, color='#F0F9FF', alpha=0.8, zorder=0)
        ax.axvspan(170, 230, color='#FEFCE8', alpha=0.8, zorder=0)
        ax.axvspan(230, 360, color='#F0FDF4', alpha=0.8, zorder=0)
        
        ax.axhline(0, color='#94A3B8', linewidth=0.9, linestyle='-', alpha=0.8, zorder=1)
        ax.axvline(70, color='#64748B', linestyle=':', linewidth=1.3, alpha=0.85, zorder=2)
        ax.axvline(170, color='#64748B', linestyle=':', linewidth=1.1, alpha=0.6, zorder=2)
        ax.axvline(230, color='#64748B', linestyle=':', linewidth=1.1, alpha=0.6, zorder=2)
        
        for ch in CHANNELS:
            c_col = ch['col']
            y_norm = data[f'{c_col}_norm']
            ax.plot(TIME_GRID, y_norm, color=ch['color'], linewidth=2.3,
                    alpha=0.95, label=f'{ch["id"]} - {ch["name"]}', zorder=3)
        
        fig.suptitle(f'Comprehensive 8-Channel Multi-Sensor Response Profile — {dev["name"]}\n'
                     f'Gas Response Profiles (C1–C8) | Time Series 0–360s, Inward IEEE Standard Style',
                     fontsize=13.5, fontweight='bold', color='#0F172A', y=0.97, va='top')
        
        ax.set_xlabel('Time (seconds)', fontsize=11, fontweight='bold', color='#1E293B', labelpad=8)
        ax.set_ylabel(r'Normalized Response $\Delta V / V_0\ (-)$', fontsize=11, fontweight='bold', color='#1E293B', labelpad=8)
        
        ax.set_xlim(0, 360)
        ax.set_xticks(np.arange(0, 361, 30))
        
        ax.tick_params(axis='both', labelsize=9.5, colors='#1E293B')
        ax.grid(True, linestyle='--', alpha=0.35, color='#CBD5E1', zorder=0)
        
        for sp in ax.spines.values():
            sp.set_edgecolor('#0F172A')
            sp.set_linewidth(1.1)
            
        # Phase header labels
        y_top = ax.get_ylim()[1]
        ax.text(35, y_top * 0.94, 'Fase 1: Baseline\n(0–70s)', ha='center', va='top', fontsize=8.5, color='#475569', fontweight='bold')
        ax.text(120, y_top * 0.94, 'Fase 2: Inlet\n(70–170s)', ha='center', va='top', fontsize=8.5, color='#0369A1', fontweight='bold')
        ax.text(200, y_top * 0.94, 'Fase Sensing\n(170–230s)', ha='center', va='top', fontsize=8.5, color='#B45309', fontweight='bold')
        ax.text(295, y_top * 0.94, 'Fase 3: Outlet Purging\n(230–360s)', ha='center', va='top', fontsize=8.5, color='#15803D', fontweight='bold')
        
        # Legend positioned nicely above axes
        fig.legend(loc='upper center', bbox_to_anchor=(0.5, 0.885), ncol=4,
                   fontsize=9.5, frameon=True, framealpha=0.95, edgecolor='#CBD5E1',
                   handlelength=2.0)
        
        out_name = f'plot_device{dev_idx+1}_combined_8sensors.png'
        out_path = os.path.join(OUTPUT_DIR, out_name)
        fig.savefig(out_path, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close(fig)
        shutil.copy(out_path, os.path.join(ARTIFACT_DIR, out_name))
        output_files.append(out_path)
        print(f"[+] Saved Combined 8-Sensor Plot: {out_name}")
        
    return output_files

# ─────────────────────────────────────────────────────────────────────────────
# 3. MASTER COMPARISON PLOT: 4x2 Grid comparing Device 1 - 5 (Matching Reference Image)
# ─────────────────────────────────────────────────────────────────────────────
def generate_comparison_grid():
    apply_plot_style()
    
    fig = plt.figure(figsize=(22, 12), facecolor='white')
    gs = gridspec.GridSpec(2, 4, figure=fig, hspace=0.36, wspace=0.25,
                           top=0.81, bottom=0.08, left=0.06, right=0.97)
    
    for i, ch in enumerate(CHANNELS):
        row, col = divmod(i, 4)
        ax = fig.add_subplot(gs[row, col])
        ax.set_facecolor('white')
        
        c_col = ch['col']
        
        # Baseline Shading (0 to 70s)
        ax.axvspan(0, 70, color='#F1F5F9', alpha=0.9, zorder=0)
        ax.axhline(0, color='#94A3B8', linewidth=0.8, linestyle='-', alpha=0.7, zorder=1)
        ax.axvline(70, color='#64748B', linestyle=':', linewidth=1.2, alpha=0.85, zorder=3)
        
        all_y = []
        for dev in DEVICES:
            dev_id = dev['id']
            y = device_data[dev_id][f'{c_col}_norm']
            ax.plot(TIME_GRID, y, color=dev['color'], linewidth=2.2, alpha=0.92, label=dev['name'], zorder=2)
            all_y.extend(y.tolist())
            
        ax.set_title(f'({letters[i]}) {ch["id"]} Normalized Response\n{ch["name"]} ({ch["target"]})',
                     fontsize=10.5, fontweight='bold', color='#0F172A', pad=6)
        
        ax.set_xlabel('Time (seconds)', fontsize=9.5, fontweight='bold', color='#1E293B')
        ax.set_ylabel(r'$\Delta V / V_0\ (-)$', fontsize=10, fontweight='bold', color='#1E293B')
        
        ax.set_xlim(0, 360)
        ax.set_xticks([0, 60, 120, 180, 240, 300, 360])
        
        y_min = min(np.nanmin(all_y), -0.05)
        y_max = max(np.nanmax(all_y), 0.1)
        margin = max((y_max - y_min) * 0.15, 0.04)
        ax.set_ylim(y_min - margin, y_max + margin)
        
        ax.tick_params(axis='both', labelsize=8.5, colors='#1E293B')
        ax.grid(True, linestyle='--', alpha=0.3, color='#CBD5E1', zorder=0)
        
        for sp in ax.spines.values():
            sp.set_edgecolor('#0F172A')
            sp.set_linewidth(1.0)
            
        ax.text(73, ax.get_ylim()[1] * 0.88, 'Inlet\n70s', fontsize=7.5,
                color='#64748B', ha='left', va='top', style='italic')
    
    fig.suptitle(
        'Comprehensive 8-Channel Multi-Sensor Responses (Channels C1–C8) — Device Comparison (Device 1–Device 5)\n'
        'Gas Response Profiles (MQ-6, MQ-135, MQ-3, MQ-7, TGS-2611, TGS-2602, TGS-2600, TGS-2620) | 0–360s, Inward IEEE Standard',
        fontsize=13.5, fontweight='bold', color='#0F172A', y=0.965, va='top'
    )
    
    legend_handles = [
        Line2D([0], [0], color=dev['color'], linewidth=2.5, label=dev['name'])
        for dev in DEVICES
    ]
    fig.legend(handles=legend_handles, loc='upper center', ncol=5,
               fontsize=10.5, frameon=True, framealpha=0.95, edgecolor='#CBD5E1',
               bbox_to_anchor=(0.5, 0.885), columnspacing=1.8, handlelength=2.2)
    
    out_name = 'plot_all_devices_comparison_grid4x2.png'
    out_path = os.path.join(OUTPUT_DIR, out_name)
    fig.savefig(out_path, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    shutil.copy(out_path, os.path.join(ARTIFACT_DIR, out_name))
    print(f"[+] Saved Master Comparison Grid Plot: {out_name}")
    return out_path

if __name__ == '__main__':
    print("Generating 5 Device Grid Plots (4x2)...")
    generate_device_grid_plots()
    
    print("\nGenerating 5 Device Combined 8-Sensor Plots...")
    generate_device_combined_plots()
    
    print("\nGenerating Master Comparison Grid Plot...")
    generate_comparison_grid()
    
    print("\nAll plots generated successfully!")
