import { useRef, useState } from 'react';
import type { Leg } from '../types';

interface Props {
  legs: Leg[];
  selectedId: string | null;
  activeLegIndex: number | null;
  onSelect: (id: string) => void;
  onAdd: () => void;
  onDelete: (id: string) => void;
  onDuplicate: (id: string) => void;
  onReorder: (fromIndex: number, toIndex: number) => void;
}

export default function LegList({ legs, selectedId, activeLegIndex, onSelect, onAdd, onDelete, onDuplicate, onReorder }: Props) {
  const dragIndex = useRef<number | null>(null);
  const [overIndex, setOverIndex] = useState<number | null>(null);

  function handleDragStart(i: number) {
    dragIndex.current = i;
  }

  function handleDragOver(e: React.DragEvent, i: number) {
    e.preventDefault(); // required to allow drop
    setOverIndex(i);
  }

  function handleDrop(i: number) {
    if (dragIndex.current !== null && dragIndex.current !== i) {
      onReorder(dragIndex.current, i);
    }
    dragIndex.current = null;
    setOverIndex(null);
  }

  function handleDragEnd() {
    dragIndex.current = null;
    setOverIndex(null);
  }

  return (
    <div className="panel leg-list">
      <div className="panel-header">
        <span>Legs</span>
        <button className="btn-add" onClick={onAdd} title="Add leg">+</button>
      </div>
      <div className="leg-items">
        {legs.length === 0 && (
          <div className="empty-hint">No legs yet. Click + to add one.</div>
        )}
        {legs.map((leg, i) => (
          <div
            key={leg.id}
            draggable
            className={[
              'leg-item',
              leg.id === selectedId ? 'selected' : '',
              overIndex === i && dragIndex.current !== i ? 'drag-over' : '',
              activeLegIndex === i ? 'sim-active' : '',
            ].join(' ').trim()}
            onClick={() => onSelect(leg.id)}
            onDragStart={() => handleDragStart(i)}
            onDragOver={e => handleDragOver(e, i)}
            onDrop={() => handleDrop(i)}
            onDragEnd={handleDragEnd}
          >
            <span className="drag-handle" title="Drag to reorder">⠿</span>
            <span className="leg-num">{i + 1}</span>
            <span className="leg-name">{leg.name}</span>
            <span className="leg-coords">
              ({leg.x.toFixed(2)}, {leg.y.toFixed(2)})
            </span>
            <div className="leg-actions" onClick={e => e.stopPropagation()}>
              <button
                className="btn-icon btn-dup"
                onClick={() => onDuplicate(leg.id)}
                title="Duplicate"
              >⧉</button>
              <button
                className="btn-icon btn-del"
                onClick={() => onDelete(leg.id)}
                title="Delete"
              >×</button>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
