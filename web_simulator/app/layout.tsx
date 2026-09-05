import type { Metadata } from 'next';
import { Geist, Geist_Mono } from 'next/font/google';
import './globals.css';

const geistSans = Geist({
  variable: '--font-geist-sans',
  subsets: ['latin'],
});

const geistMono = Geist_Mono({
  variable: '--font-geist-mono',
  subsets: ['latin'],
});

export const metadata: Metadata = {
  title: 'Robo · 햄스터 1대·비버 3대 실시간 좌표 추적',
  description:
    '햄스터 1대·비버 3대 운반 모의, 드론 선택, AprilTag 좌표·방향과 색 물체 JSONL 관측 확인. 실제 하드웨어와 분리된 연습 시뮬레이터.',
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="ko">
      <body
        className={`${geistSans.variable} ${geistMono.variable} antialiased`}
      >
        {children}
      </body>
    </html>
  );
}
