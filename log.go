package slam

import (
	"fmt"
	"io"
	"log"
	"os"
)

// Logger is the minimal logging surface used by the pipeline. Tests plug in
// a capturing logger; production uses NewTextLogger.
type Logger interface {
	// Framef logs a per-frame decision with its inputs and key numbers.
	Framef(format string, args ...any)
	// Loopf logs loop-closure accept/soft/reject decisions.
	Loopf(format string, args ...any)
	// Optf logs optimizer iterations and convergence.
	Optf(format string, args ...any)
	// Localizef logs cross-session global localization attempts.
	Localizef(format string, args ...any)
}

// NewTextLogger writes human-readable lines to w (nil => stderr).
func NewTextLogger(w io.Writer) Logger {
	if w == nil {
		w = os.Stderr
	}
	l := log.New(w, "slam ", log.LstdFlags|log.Lmicroseconds)
	return &textLogger{l: l}
}

type textLogger struct{ l *log.Logger }

func (t *textLogger) Framef(f string, a ...any) { t.l.Printf("frame "+f, a...) }
func (t *textLogger) Loopf(f string, a ...any)  { t.l.Printf("loop  "+f, a...) }
func (t *textLogger) Optf(f string, a ...any)   { t.l.Printf("opt   "+f, a...) }
func (t *textLogger) Localizef(f string, a ...any) {
	t.l.Printf("loc   "+f, a...)
}

// NopLogger discards everything.
func NopLogger() Logger { return nopLogger{} }

type nopLogger struct{}

func (nopLogger) Framef(string, ...any)    {}
func (nopLogger) Loopf(string, ...any)     {}
func (nopLogger) Optf(string, ...any)      {}
func (nopLogger) Localizef(string, ...any) {}

// SliceLogger captures lines, useful in tests for assertions and for the
// requirement that test logs print inputs, decisions and errors.
type SliceLogger struct {
	FrameLines []string
	LoopLines  []string
	OptLines   []string
	LocLines   []string
}

func (s *SliceLogger) Framef(f string, a ...any) {
	s.FrameLines = append(s.FrameLines, fmt.Sprintf(f, a...))
}
func (s *SliceLogger) Loopf(f string, a ...any) {
	s.LoopLines = append(s.LoopLines, fmt.Sprintf(f, a...))
}
func (s *SliceLogger) Optf(f string, a ...any) {
	s.OptLines = append(s.OptLines, fmt.Sprintf(f, a...))
}
func (s *SliceLogger) Localizef(f string, a ...any) {
	s.LocLines = append(s.LocLines, fmt.Sprintf(f, a...))
}
