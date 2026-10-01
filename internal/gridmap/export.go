package gridmap

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"strings"
)

// WritePGM writes the occupancy map as an 8-bit PGM (P5). Occupied cells are
// dark (0), free cells light (255), unknown cells gray (205). Deterministic.
func (m *Map) WritePGM(path string) error {
	m.mu.RLock()
	defer m.mu.RUnlock()
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriter(f)
	defer w.Flush()
	fmt.Fprintf(w, "P5\n%d %d\n255\n", m.size, m.size)
	buf := make([]byte, m.size)
	for y := m.size - 1; y >= 0; y-- {
		for x := 0; x < m.size; x++ {
			lo := m.logOdds[y*m.size+x]
			var v byte = 205
			switch {
			case lo > 0.2:
				v = 0
			case lo < -0.2:
				v = 254
			}
			buf[x] = v
		}
		if _, err := w.Write(buf); err != nil {
			return err
		}
	}
	return nil
}

// WriteCSV writes occupied cell centers as "x,y" rows.
func (m *Map) WriteCSV(path string) error {
	m.mu.RLock()
	defer m.mu.RUnlock()
	var sb strings.Builder
	for y := 0; y < m.size; y++ {
		for x := 0; x < m.size; x++ {
			if m.logOdds[y*m.size+x] > 0 {
				p := m.CellWorld(x, y)
				fmt.Fprintf(&sb, "%.4f,%.4f\n", p.X, p.Y)
			}
		}
	}
	return os.WriteFile(path, []byte(sb.String()), 0o644)
}

var _ io.Writer = (*bufio.Writer)(nil)
