#ifndef HIGHLIGHTER_H_
#define HIGHLIGHTER_H_

#include <QSyntaxHighlighter>

#ifdef _QCODE_EDIT_
#include "qdocument.h"
#endif

class Highlighter : public QSyntaxHighlighter
{
public:
  enum State { NORMAL=-1, QUOTE,COMMENT };
  enum Mode { NORMAL_MODE, ERROR_MODE };

#ifdef _QCODE_EDIT_
	Highlighter(QDocument *parent, Mode mode);
#else
	Highlighter(QTextDocument *parent, Mode mode);
#endif
	void highlightBlock(const QString &text);

	Mode mode() const { return mode_; }

private:
	Mode mode_;

  QStringList Operators;
  QStringList KeyWords;
  QStringList Primitives3D;
  QStringList Primitives2D;
  QStringList Transforms;
  QStringList Imports;
  QStringList Functions;

  QTextCharFormat OperatorStyle;
  QTextCharFormat CommentStyle;
  QTextCharFormat QuoteStyle;
  QTextCharFormat KeyWordStyle;
  QTextCharFormat PrimitiveStyle3D;
  QTextCharFormat PrimitiveStyle2D;
  QTextCharFormat TransformStyle;
  QTextCharFormat ImportStyle;
  QTextCharFormat FunctionStyle;
  QTextCharFormat ErrorStyle;
};

#endif
